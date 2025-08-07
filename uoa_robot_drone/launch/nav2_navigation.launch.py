from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

map_path = os.path.join(get_package_share_directory('uoa_robot_drone'), 'maps', 'uoa_robot_drone_vslam_202508051615.yaml')
param_file_path = os.path.join(get_package_share_directory('uoa_robot_drone'), 'config', 'nav2_controller_params.yaml')
print(f"Controller parameters file path: {param_file_path}")

robot_base_frame = {'robot_base_frame': 'zed_camera_link'}
remappings = [
    ('odom', '/vo/odom'),
    ('cmd_vel', '/tb3/cmd_vel'),
]

def generate_launch_description():
   
     # Path to RViz config (reuse the demo config)
    cfg_rviz = os.path.join(
        get_package_share_directory('uoa_robot_drone'),
        'config',
        'rviz_nav2.rviz' 
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='map file to use for navigation'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument('rviz_cfg',       default_value=cfg_rviz,
                               description='Path to RViz2 config file'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=remappings
        ),
        # Node(
        #     package='nav2_amcl', # AMCL (Adaptive Monte Carlo Localization) node
        #     executable='amcl',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     arguments=['--ros-args', '--log-level', 'warn'],
        # ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        robot_base_frame,
            ],
            remappings=remappings
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                robot_base_frame,
            ],
            remappings=remappings,
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[param_file_path],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=remappings
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        robot_base_frame],
            remappings=remappings
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=remappings
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [
                    'map_server',
                    # 'amcl',
                    'behavior_server',
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
        Node(
            package='rviz2', executable='rviz2', output='screen',
            name='rviz2',
            arguments=[['-d'], [LaunchConfiguration('rviz_cfg')]]
        ),
    ])