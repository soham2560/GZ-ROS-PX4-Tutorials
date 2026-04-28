import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('px4_rrt_nav')
    map_yaml_file = os.path.join(pkg_dir, 'maps', 'map.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'px4_rrt.rviz')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'quadrotor.urdf')

    # Read the URDF file to pass it as a string to the publisher
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Start Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}, 
                        {'topic_name': "map"}, 
                        {'frame_id': "map"}]
        ),
        
        # 2. Lifecycle manager to automatically start the map server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # 3. Custom RRT Planner Node
        Node(
            package='px4_rrt_nav',
            executable='rrt_planner',
            name='rrt_planner',
            output='screen',
            parameters=[{'inflation_radius': 0.40}, {'step_size': 0.02}]
        ),

        # 4. Offboard Control Node (Publishes dynamic map -> base_link TF)
        Node(
            package='px4_rrt_nav',
            executable='offboard_control',
            name='offboard_control',
            output='screen'
        ),

        # 5. Robot State Publisher (Publishes static base_link -> rotor TFs & /robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 6. Open RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])