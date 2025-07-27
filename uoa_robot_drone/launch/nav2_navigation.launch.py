from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

map_path = os.path.join(get_package_share_directory('uoa_robot_drone'), 'maps', 'uoa_robot_drone_map_202507231952.yaml')
param_file_path = os.path.join(get_package_share_directory('uoa_robot_drone'), 'config', 'nav2_controller_params.yaml')

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='map file to use for navigation'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            namespace='navi',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='nav2_amcl', # AMCL (Adaptive Monte Carlo Localization) node
            executable='amcl',
            namespace='navi',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace='navi',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace='navi',
            name='controller_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        param_file_path]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace='navi',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            namespace='navi',
            name='waypoint_follower',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace='navi',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'
        # )
    ])