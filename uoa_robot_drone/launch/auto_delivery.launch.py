from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare the delivery_ready argument
    declare_delivery_ready_arg = DeclareLaunchArgument(
            'delivery_ready',
            default_value='true',
            description='Set the state of delivery readiness'
    )

    return LaunchDescription([
        declare_delivery_ready_arg,
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

