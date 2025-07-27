'''
uoa_robot_drone/robot_position_listener.py
This script listens to the robot's position and checks if it is close enough to a specified goal position and orientation.
If the robot is close enough, it publishes a delivery signal.
It also provides a service to update the goal position and orientation dynamically.
This code is part of the uoa_robot_drone package, which is used for autonomous delivery.
Copyright (c) University of Aberdeen.
This code is licensed under the Apache-2.0 license.

Author: Jian Chen
Email: j.chen3.24@abdn.ac.uk

Usage:
1. Start the ROS 2 node:
   ros2 run uoa_robot_drone robot_position_listener
2. To update the goal position and orientation, call the service:
   ros2 service call /set_robot_goal_pose uoa_robot_drone/srv/SetRobotGoalPose "{position_x: 1.0, position_y: 2.0,
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import math
from uoa_robot_interfaces.srv import SetRobotGoalPose  # Import the service definition

class RobotPositionListener(Node):
    def __init__(self):
        super().__init__('robot_position_listener')
        self.get_logger().info('Robot Position Listener Node is starting...')
        
        # Declare parameters with default values
        self.declare_parameter('goal_position_x', 1.2)
        self.declare_parameter('goal_position_y', 1.4)
        self.declare_parameter('goal_position_z', 0.0)
        self.declare_parameter('goal_orientation_x', 0.0)
        self.declare_parameter('goal_orientation_y', 0.0)
        self.declare_parameter('goal_orientation_z', 3.14/2)
        self.declare_parameter('goal_orientation_w', 0.2)
        self.declare_parameter('distance_threshold', 0.1)
        self.declare_parameter('orientation_threshold', 0.1)

        # Get parameters
        self.goal_position_x = self.get_parameter('goal_position_x').value
        self.goal_position_y = self.get_parameter('goal_position_y').value
        self.goal_position_z = self.get_parameter('goal_position_z').value
        self.goal_orientation_x = self.get_parameter('goal_orientation_x').value
        self.goal_orientation_y = self.get_parameter('goal_orientation_y').value
        self.goal_orientation_z = self.get_parameter('goal_orientation_z').value
        self.goal_orientation_w = self.get_parameter('goal_orientation_w').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.orientation_threshold = self.get_parameter('orientation_threshold').value

        self.get_logger().info(f'Goal Position: ({self.goal_position_x}, {self.goal_position_y}, {self.goal_position_z})')
        self.get_logger().info(f'Goal Orientation: ({self.goal_orientation_x}, {self.goal_orientation_y}, {self.goal_orientation_z}, {self.goal_orientation_w})')

        self.delivery_signal = Bool()
        self.delivery_signal.data = False

        # Create a publisher to publish the delivery signal
        self.delivery_signal_publisher = self.create_publisher(Bool, 'delivery_signal', 10)

        # Create a subscription to the robot's pose
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.pose_callback, 10)
        
        self.srv = self.create_service(SetRobotGoalPose, 'set_robot_goal_pose', self.set_robot_goal_pose_callback)

    def set_robot_goal_pose_callback(self, request, response):
        # Update the goal position and orientation based on the service request
        self.goal_position_x = request.position_x
        self.goal_position_y = request.position_y
        self.goal_orientation_z = request.orientation_z
        self.goal_orientation_w = request.orientation_w

        # Log the updated goal pose
        self.get_logger().info(f'Updated Goal Position: ({self.goal_position_x}, {self.goal_position_y})')
        self.get_logger().info(f'Updated Goal Orientation: ({self.goal_orientation_z}, {self.goal_orientation_w})')

        response.success = True
        response.message = 'Goal pose updated successfully'
        return response

    def pose_callback(self, msg):
        # Extract the robot's current position and orientation
        current_position = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation

        # Calculate the distance to the goal position
        distance_to_goal = math.sqrt(
            (current_position.x - self.goal_position_x) ** 2 +
            (current_position.y - self.goal_position_y) ** 2 +
            (current_position.z - self.goal_position_z) ** 2
        )

        # Calculate the orientation difference
        orientation_difference = math.sqrt(
            (current_orientation.x - self.goal_orientation_x) ** 2 +
            (current_orientation.y - self.goal_orientation_y) ** 2 +
            (current_orientation.z - self.goal_orientation_z) ** 2 +
            (current_orientation.w - self.goal_orientation_w) ** 2
        )

        # Log the current position, orientation, and distance to goal
        self.get_logger().debug(f'Current Position: ({current_position.x}, {current_position.y}, {current_position.z})')
        self.get_logger().debug(f'Current Orientation: ({current_orientation.x}, {current_orientation.y}, {current_orientation.z}, {current_orientation.w})')
        self.get_logger().debug(f'Distance to Goal: {distance_to_goal}')
        self.get_logger().debug(f'Orientation Difference: {orientation_difference}')

        # Check if the robot is close enough to the goal position and orientation
        if distance_to_goal < self.distance_threshold and orientation_difference < self.orientation_threshold:
            self.get_logger().debug('Robot is at the goal position and orientation.')
            self.delivery_signal.data = True
        else:
            self.get_logger().debug('Robot is not at the goal position and orientation.')
            self.delivery_signal.data = False

        # Publish the delivery signal
        self.delivery_signal_publisher.publish(self.delivery_signal)                                                           

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(node.get_name() + ' has been stopped by user.')
    finally:
        node.get_logger().info('Shutting down ' + node.get_name() + ' node.')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# End of file: uoa_robot_drone/uoa_robot_drone/robot_position_listener