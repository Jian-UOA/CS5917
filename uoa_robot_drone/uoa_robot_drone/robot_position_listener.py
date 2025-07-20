import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import math

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
        self.get_logger().info(f'Current Position: ({current_position.x}, {current_position.y}, {current_position.z})')
        self.get_logger().info(f'Current Orientation: ({current_orientation.x}, {current_orientation.y}, {current_orientation.z}, {current_orientation.w})')
        self.get_logger().info(f'Distance to Goal: {distance_to_goal}')
        self.get_logger().info(f'Orientation Difference: {orientation_difference}')

        # Check if the robot is close enough to the goal position and orientation
        if distance_to_goal < self.distance_threshold and orientation_difference < self.orientation_threshold:
            self.get_logger().info('Robot is at the goal position and orientation.')
            self.delivery_signal.data = True
        else:
            self.get_logger().info('Robot is not at the goal position and orientation.')
            self.delivery_signal.data = False

        # Publish the delivery signal
        self.delivery_signal_publisher.publish(self.delivery_signal)                                                           

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionListener()
    try:
        rclpy.spin(node)
    except keyboardInterrupt:
        node.get_logger().info(node.get_name() + ' has been stopped by user.')
    finally:
        node.get_logger().info('Shutting down' + node.get_name() + 'node.')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# End of file: uoa_robot_drone/uoa_robot_drone/robot_position_listener