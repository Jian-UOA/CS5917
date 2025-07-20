import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ConvertImageFormat(Node):
    def __init__(self):
        super().__init__('convert_image_format')
        self.get_logger().info("ConvertImageFormat node started.")
        
        # Declare parameters with default values
        self.declare_parameter('from_format', 'bgra8')
        self.declare_parameter('to_format', 'bgr8')
        self.declare_parameter('subscription', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('publisher', '/zed/zed_node/left/image_rect_bgr')
        
        # Get parameter values
        self.from_format = self.get_parameter('from_format').get_parameter_value().string_value
        self.to_format = self.get_parameter('to_format').get_parameter_value().string_value
        subscription_topic = self.get_parameter('subscription').get_parameter_value().string_value
        publisher_topic = self.get_parameter('publisher').get_parameter_value().string_value
        
        self.get_logger().info(f"Converting from {self.from_format} to {self.to_format}")
        self.get_logger().info(f"Subscribing to: {subscription_topic}")
        self.get_logger().info(f"Publishing to: {publisher_topic}")
        
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            subscription_topic,
            self.callback,
            10)
        self.pub = self.create_publisher(
            Image,
            publisher_topic,
            10)
        self.frame_count = 0

    def callback(self, msg):
        try:
            if msg.encoding != self.from_format:
                self.get_logger().warning(f"[convert_image_format] Unexpected encoding: {msg.encoding} (expect {self.from_format})")
                return
            if not msg.data or msg.height == 0 or msg.width == 0:
                self.get_logger().error(f"[convert_image_format] Received empty image: stamp={msg.header.stamp}, frame_id={msg.header.frame_id}")
                return
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.from_format)
            if cv_img is None or cv_img.size == 0:
                self.get_logger().error("[convert_image_format] cv_img is empty after conversion!")
                return
            bgr_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
            out_msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding=self.to_format)
            out_msg.header = msg.header
            self.pub.publish(out_msg)
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f"[convert_image_format] Converted {self.frame_count} frames, latest stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        except Exception as e:
            self.get_logger().error(f"[convert_image_format] Exception: {repr(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ConvertImageFormat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ConvertImageFormat node interrupted by user (Ctrl-C).")
    finally:
        node.get_logger().info("ConvertImageFormat node shutting down.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
