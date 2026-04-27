#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OffboardControl(Node):
    """Professional node for controlling a vehicle and broadcasting TF."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PX4 Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # PX4 Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Path Subscriber
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)

        # --- NEW: TF Broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        self.current_path = []
        self.path_active = False
        self.current_yaw = 0.0
        
        self.lookahead_distance = 1.0  
        self.max_speed = 0.5           
        self.dt = 0.1                  
        
        self.current_setpoint = [0.0, 0.0, self.altitude] 
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def path_callback(self, msg: Path):
        self.get_logger().info("New Global Path Received! Executing smooth trajectory.")
        
        # --- FIXED: CONVERT INCOMING ENU PATH TO INTERNAL NED PATH ---
        # RViz Path (ENU): p.pose.position.x is East, p.pose.position.y is North
        # PX4 Flight (NED): position[0] is North, position[1] is East
        self.current_path = [[p.pose.position.y, p.pose.position.x, self.altitude] for p in msg.poses]
        
        self.path_active = True
        if self.vehicle_local_position.timestamp != 0:
            self.current_setpoint = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.altitude]

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

        # --- FIXED: CONVERT INTERNAL NED POSITION TO ENU FOR RVIZ TF ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link' # The drone's frame
        
        # PX4 msg is NED (x=North, y=East, z=Down). RViz wants ENU (x=East, y=North, z=Up)
        t.transform.translation.x = float(msg.y)  # East
        t.transform.translation.y = float(msg.x)  # North
        t.transform.translation.z = float(-msg.z) # Up
        
        # Convert Heading (Euler Yaw) to Quaternion for RViz
        yaw_enu = -msg.heading + (math.pi / 2.0) 
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw_enu / 2.0)
        t.transform.rotation.w = math.cos(yaw_enu / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, setpoint, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = setpoint
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def calculate_local_setpoint(self):
        if not self.path_active or len(self.current_path) == 0:
            return self.current_setpoint, self.current_yaw 

        px, py = self.vehicle_local_position.x, self.vehicle_local_position.y
        
        target_pt = self.current_path[-1] 
        for i, pt in enumerate(self.current_path):
            dist = math.hypot(pt[0] - px, pt[1] - py)
            if dist > self.lookahead_distance:
                target_pt = pt
                self.current_path = self.current_path[i:] 
                break

        dx = target_pt[0] - self.current_setpoint[0]
        dy = target_pt[1] - self.current_setpoint[1]
        dist_to_target = math.hypot(dx, dy)
        
        max_step = self.max_speed * self.dt
        
        if dist_to_target < max_step:
            self.current_setpoint = [target_pt[0], target_pt[1], self.altitude]
            if len(self.current_path) == 1 and dist_to_target < 0.2:
                self.get_logger().info("Goal Reached. Hovering.")
                self.path_active = False
        else:
            self.current_setpoint[0] += (dx / dist_to_target) * max_step
            self.current_setpoint[1] += (dy / dist_to_target) * max_step

        if dist_to_target > 0.01:
            self.current_yaw = math.atan2(dy, dx)

        return self.current_setpoint, self.current_yaw

    def timer_callback(self) -> None:
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.vehicle_local_position.timestamp > 0:
            setpoint, yaw = self.calculate_local_setpoint()
            self.publish_trajectory_setpoint(setpoint, yaw)
        else:
            self.publish_trajectory_setpoint([0.0, 0.0, self.altitude], 0.0)

def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()