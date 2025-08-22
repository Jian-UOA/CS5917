'''
auto_delivery.launch.py
Launch file for the automated delivery system

This launch file starts the auto_delivery node and the robot_position_listener node.
It includes launch arguments to enable or disable subscription to the /goal_pose topic,
as well as to set the initial state of the delivery process.

Author: Jian Chen
Email: j.chen3.24@abdn.ac.uk

Prerequisites:
- ROS2 installed
- Tello drone connected to the same network
- Required Python packages installed

Example usage:
- ros2 launch uoa_robot_drone auto_delivery.launch.py
- ros2 service call /uoa/auto_delivery/reset_delivery_ready std_srvs/srv/SetBool "{data: true}"
- ros2 topic pub /uoa/delivery_signal std_msgs/Bool "{data: true}"
- ros2 topic pub /uoa/delivery_signal std_msgs/Bool "{data: false}"
'''

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    subscribe_goal_pose = DeclareLaunchArgument(
        'subscribe_goal_pose',
        default_value='true',
        description='Enable or disable subscription to /goal_pose topic'
    )

    # Declare the delivery_ready argument
    declare_delivery_ready_arg = DeclareLaunchArgument(
            'delivery_ready',
            default_value='true',
            description='Set the state of delivery readiness'
    )
    localization = DeclareLaunchArgument(
            'localization',
            default_value='true',
            description='Enable or disable localization'
    )

    return LaunchDescription([
        subscribe_goal_pose,
        declare_delivery_ready_arg,
        localization,
        Node(
            condition=IfCondition(LaunchConfiguration('localization')),
            package='uoa_robot_drone', executable='robot_position_listener', output='screen',
            namespace='uoa',
            name='robot_position_listener',
            # Use parameters to set goal position and orientation
            parameters=[{
                'subscribe_goal_pose': LaunchConfiguration('subscribe_goal_pose'),
                'localization': LaunchConfiguration('localization'),
                'delivery_signal/rate': 1.0,  # Default rate for publishing delivery signal
                'goal_position_x': 0.158171,
                'goal_position_y': 1.83296,
                'goal_position_z': 0.0,
                'goal_orientation_x': 0.0,
                'goal_orientation_y': 0.0,
                'goal_orientation_z': -0.999993,  
                'goal_orientation_w': 0.00377529,
                'distance_threshold': 0.1,
                'orientation_threshold': 0.1
            }]
        ),
        Node(
            package='uoa_robot_drone',
            executable='auto_delivery',
            namespace='uoa',
            name='auto_delivery',
            output='screen',
            parameters=[{'delivery_ready': LaunchConfiguration('delivery_ready')}],
            remappings=[
                ('delivery_signal', '/uoa/delivery_signal')
            ]
        )
    ])

