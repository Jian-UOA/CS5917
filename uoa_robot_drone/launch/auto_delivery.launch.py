from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

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
        declare_delivery_ready_arg,
        localization,
        Node(
            condition=IfCondition(LaunchConfiguration('localization')),
            package='uoa_robot_drone', executable='robot_position_listener', output='screen',
            namespace='uoa',
            name='robot_position_listener',
            # Use parameters to set goal position and orientation
            parameters=[{
                'goal_position_x': 0.66,
                'goal_position_y': 0.78,
                'goal_position_z': 0.0,
                'goal_orientation_x': 0.0,
                'goal_orientation_y': 0.0,
                'goal_orientation_z': 0.58,  
                'goal_orientation_w': 0.80,
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

