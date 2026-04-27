#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
import math
import random
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import Marker

class RRTNode:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.cost = 0.0
        self.parent = None
        self.children = set()

def dist(n1: RRTNode, n2: RRTNode) -> float:
    return math.hypot(n1.x - n2.x, n1.y - n2.y)

def propagate_cost_to_leaves(start_node: RRTNode):
    queue = [start_node]
    while queue:
        curr = queue.pop(0)
        for child in curr.children:
            child.cost = curr.cost + dist(curr, child)
            queue.append(child)

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        
        self.declare_parameter('inflation_radius', 0.25) 
        self.declare_parameter('max_iter', 10000)      
        self.declare_parameter('step_size', 0.01)      # Interpret explicitly as meters
        self.declare_parameter('search_radius', 2.0)  # Interpret explicitly as meters
        self.declare_parameter('goal_bias', 0.001)      
        
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.max_iter = self.get_parameter('max_iter').value
        self.step_size = self.get_parameter('step_size').value
        self.search_radius = self.get_parameter('search_radius').value
        self.goal_bias = self.get_parameter('goal_bias').value

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_callback, telemetry_qos)
        
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', map_qos)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.tree_pub = self.create_publisher(Marker, '/rrt_tree', 10)

        self.inflated_grid = None
        self.map_info = None
        self.start_pose = (0.0, 0.0) 

        self.inflated_msg_cache = None
        self.map_timer = self.create_timer(2.0, self.publish_cached_map)

        self.get_logger().info("RRT* Planner Active! Use '2D Goal Pose' in RViz to plan.")

    def pos_callback(self, msg: VehicleLocalPosition):
        if msg.timestamp > 0:
            # CONVERT PX4 NED TO ROS ENU: East = msg.y, North = msg.x
            self.start_pose = (msg.y, msg.x)

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        binary_map = np.zeros_like(grid, dtype=np.uint8)
        binary_map[(grid > 50) | (grid == -1)] = 255 

        inflation_pixels = int(self.inflation_radius / self.map_info.resolution)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflation_pixels*2+1, inflation_pixels*2+1))
        self.inflated_grid = cv2.dilate(binary_map, kernel)

        inflated_msg = OccupancyGrid()
        inflated_msg.header = msg.header
        inflated_msg.info = msg.info
        out_grid = np.zeros_like(self.inflated_grid, dtype=np.int8)
        out_grid[self.inflated_grid == 255] = 100
        inflated_msg.data = out_grid.flatten().tolist()
        
        self.inflated_msg_cache = inflated_msg
        self.publish_cached_map()

    def publish_cached_map(self):
        if self.inflated_msg_cache is not None:
            self.inflated_msg_cache.header.stamp = self.get_clock().now().to_msg()
            self.inflated_map_pub.publish(self.inflated_msg_cache)

    def goal_callback(self, msg: PoseStamped):
        if self.inflated_grid is None:
            self.get_logger().warn("Waiting for map...")
            return

        goal_world = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Planning RRT* from {self.start_pose} to: {goal_world}")

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.inflated_grid.shape

        start_px = ((self.start_pose[0] - ox) / res, (self.start_pose[1] - oy) / res)
        goal_px  = ((goal_world[0] - ox) / res, (goal_world[1] - oy) / res)

        fail = False
        if not (0 <= int(start_px[0]) < w and 0 <= int(start_px[1]) < h):
            self.get_logger().error(f"FAIL: Drone px {start_px} outside map!")
            fail = True
        elif self.inflated_grid[int(start_px[1]), int(start_px[0])] != 0:
            self.get_logger().error("FAIL: Drone is currently INSIDE an obstacle!")
            fail = True

        if not (0 <= int(goal_px[0]) < w and 0 <= int(goal_px[1]) < h):
            self.get_logger().error("FAIL: Goal is outside the map boundaries!")
            fail = True
        elif self.inflated_grid[int(goal_px[1]), int(goal_px[0])] != 0:
            self.get_logger().error("FAIL: Goal clicked is INSIDE an obstacle!")
            fail = True

        if fail: return

        # Scale step sizes from meters to pixels using grid resolution
        step_size_px = self.step_size / res
        search_radius_px = self.search_radius / res

        raw_path_px, tree_nodes = self.rrt_star(start_px, goal_px, self.inflated_grid, step_size_px, search_radius_px)
        self.publish_tree(tree_nodes, msg.header.frame_id)
        
        if raw_path_px:
            pruned_path_px = self.prune_path(raw_path_px, self.inflated_grid)
            path_world = [(ox + px*res, oy + py*res) for (px, py) in pruned_path_px]
            self.publish_path(path_world, msg.header.frame_id)
            self.get_logger().info(f"Path optimal! Waypoints: {len(raw_path_px)} -> {len(pruned_path_px)}")
        else:
            self.get_logger().error("RRT* failed to reach goal within max_iter.")

    def publish_tree(self, node_list, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.05 
        marker.color.a = 0.5  
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        for node in node_list:
            if node.parent is not None:
                p_parent = Point(x=float(ox + node.parent.x * res), y=float(oy + node.parent.y * res), z=0.1)
                p_node = Point(x=float(ox + node.x * res), y=float(oy + node.y * res), z=0.1)
                marker.points.append(p_parent)
                marker.points.append(p_node)

        self.tree_pub.publish(marker)

    def line_of_sight(self, grid: np.ndarray, x0: float, y0: float, x1: float, y1: float) -> bool:
        h, w = grid.shape
        x0, y0 = int(round(x0)), int(round(y0))
        x1, y1 = int(round(x1)), int(round(y1))
        dx = abs(x1 - x0);  sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            if not (0 <= x < w and 0 <= y < h): return False
            if grid[y, x] != 0: return False
            if x == x1 and y == y1: return True
            e2 = 2 * err
            if e2 >= dy: err += dy; x += sx
            if e2 <= dx: err += dx; y += sy

    def rrt_star(self, start_coord, goal_coord, grid: np.ndarray, step_size_px: float, search_radius_px: float):
        h, w = grid.shape
        start_node = RRTNode(start_coord[0], start_coord[1])
        goal_node  = RRTNode(goal_coord[0], goal_coord[1])
        node_list = [start_node]
        best_goal_node = None

        for i in range(self.max_iter):
            if random.random() < self.goal_bias:
                rnd = RRTNode(goal_node.x, goal_node.y)
            else:
                rnd = RRTNode(random.uniform(0, w - 1), random.uniform(0, h - 1))
                
            nearest_node = min(node_list, key=lambda n: dist(n, rnd))
            d = dist(nearest_node, rnd)
            
            if d <= step_size_px:
                new_node = RRTNode(rnd.x, rnd.y)
            else:
                theta = math.atan2(rnd.y - nearest_node.y, rnd.x - nearest_node.x)
                new_node = RRTNode(nearest_node.x + step_size_px * math.cos(theta),
                                   nearest_node.y + step_size_px * math.sin(theta))
                
            if not (0 <= int(new_node.x) < w and 0 <= int(new_node.y) < h): continue
            if not self.line_of_sight(grid, nearest_node.x, nearest_node.y, new_node.x, new_node.y): continue
                
            near_nodes = [n for n in node_list if dist(n, new_node) <= search_radius_px]
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + dist(nearest_node, new_node)
            
            for near_node in near_nodes:
                potential_cost = near_node.cost + dist(near_node, new_node)
                if potential_cost < new_node.cost:
                    if self.line_of_sight(grid, near_node.x, near_node.y, new_node.x, new_node.y):
                        new_node.parent = near_node
                        new_node.cost = potential_cost
                        
            new_node.parent.children.add(new_node)
            node_list.append(new_node)
            
            for near_node in near_nodes:
                potential_cost = new_node.cost + dist(new_node, near_node)
                if potential_cost < near_node.cost:
                    if self.line_of_sight(grid, new_node.x, new_node.y, near_node.x, near_node.y):
                        near_node.parent.children.remove(near_node)
                        near_node.parent = new_node
                        near_node.cost = potential_cost
                        near_node.parent.children.add(near_node)
                        propagate_cost_to_leaves(near_node)

            if dist(new_node, goal_node) <= step_size_px:
                if self.line_of_sight(grid, new_node.x, new_node.y, goal_node.x, goal_node.y):
                    if best_goal_node is None or new_node.cost + dist(new_node, goal_node) < best_goal_node.cost:
                        final_node = RRTNode(goal_node.x, goal_node.y)
                        final_node.parent = new_node
                        final_node.cost = new_node.cost + dist(new_node, final_node)
                        best_goal_node = final_node
                        break 
                        
        if best_goal_node is None: return None, node_list 
            
        path = []
        curr = best_goal_node
        while curr is not None:
            path.append((curr.x, curr.y))
            curr = curr.parent
        return path[::-1], node_list

    def prune_path(self, path, grid: np.ndarray):
        if not path or len(path) <= 2: return path
        pruned = [path[0]]
        curr_idx = 0
        while curr_idx < len(path) - 1:
            furthest_visible_idx = curr_idx + 1
            for look_ahead_idx in range(len(path) - 1, curr_idx, -1):
                x0, y0 = pruned[-1]
                x1, y1 = path[look_ahead_idx]
                if self.line_of_sight(grid, x0, y0, x1, y1):
                    furthest_visible_idx = look_ahead_idx
                    break 
            pruned.append(path[furthest_visible_idx])
            curr_idx = furthest_visible_idx
        return pruned

    def publish_path(self, path_world, frame_id):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        for px, py in path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.1 
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()