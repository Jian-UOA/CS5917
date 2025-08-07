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

