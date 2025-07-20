"""
uoa_robot_drone - A ROS2 package for controlling a Tello drone for automated delivery
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from djitellopy import Tello
import time
import cv2
import apriltag
import numpy as np
from std_srvs.srv import SetBool

class AutoDelivery(Node):
    def __init__(self):
        super().__init__('auto_delivery')
        self.keep_alive_timer = self.create_timer(10.0, self.keep_alive_callback)
        self.delivery_subscriber = self.create_subscription(
            Bool,
            'delivery_signal',
            self.delivery_callback,
            10
        )
        self.delivery_ready = True
        self.srv = self.create_service(SetBool, 'reset_delivery_ready', self.reset_delivery_ready_callback)
        self.get_logger().info('delivery_ready default value: True')

    def delivery_callback(self, msg):
        if not self.delivery_ready:
            self.get_logger().info('Delivery not ready, waiting for reset...')
            return
        if not msg.data:
            self.get_logger().info('No delivery signal received, waiting for delivery signal...')
            return
        self.get_logger().info(f'Received delivery signal: {msg.data}')
        if not hasattr(self, 'tello'):
            try:
                self.connect_to_tello()
            except Exception as e:
                self.get_logger().error(f'Failed to connect to Tello: {e}')
                return
        try:
            if msg.data:
                self.get_logger().info('Starting delivery...')
                self.tello.takeoff()
                time.sleep(5)
                self.tello.move_up(30)
                time.sleep(2)
                self.tello.move_forward(70)
                time.sleep(2)
                self.tello.rotate_clockwise(180)
                time.sleep(2)
                self.tello.land()
                time.sleep(2)
                self.get_logger().info('Delivery completed.')
                time.sleep(2)
                self.tello.takeoff()
                self.get_logger().info('Starting return to base...')
                time.sleep(5)
                self.tello.move_forward(70)
                time.sleep(2)
                self.tello.rotate_clockwise(180)
                time.sleep(3)
                self.return_and_land_with_apriltag()
                self.get_logger().info('Return to base completed.')
                self.delivery_ready = False
                self.get_logger().info('delivery_ready set to False, waiting for reset...')
            else:
                self.get_logger().info('No delivery signal received, waiting for delivery signal...')
        except Exception as e:
            self.get_logger().error(f'Error during delivery: {e}')
            self.tello.end()
            del self.tello  # Attempt to reset the Tello object
            self.get_logger().info('Tello object reset, waiting for next delivery signal.')

    def return_and_land_with_apriltag(self, max_attempts=15, move_step=20, tol=30):
        """
        Use AprilTag to align and land precisely.
        This method uses the Tello's camera to detect an AprilTag and adjust the drone's
        max_attempts: Maximum number of attempts to align with the AprilTag
        move_step: Step size for adjustments (cm)
        tol: Tolerance in image pixels for alignment
        """
        self.get_logger().info('Starting AprilTag visual landing...')
        self.tello.streamon()
        time.sleep(5)
        try:
            detector = apriltag.Detector()
        except Exception as e:
            self.get_logger().error(f'AprilTag Detector initialization failed: {e}')
            self.tello.streamoff()
            self.tello.land()
            return
        for attempt in range(max_attempts):
            for _ in range(5):
                frame = self.tello.get_frame_read().frame
                if frame is not None:
                    break
                self.get_logger().warning('Failed to grab video frame, retrying...')
                time.sleep(1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray)
            tag0 = None
            for tag in tags:
                if tag.tag_id == 0:
                    tag0 = tag
                    break
            if tag0:
                center_x, center_y = tag0.center
                img_center_x = frame.shape[1] // 2
                img_center_y = frame.shape[0] // 2
                dx = int(center_x - img_center_x)
                dy = int(center_y - img_center_y)
                self.get_logger().info(f"AprilTag ID=0 offset dx={dx}, dy={dy}")
                moved = False
                if abs(dx) > tol:
                    if dx > 0:
                        self.tello.move_right(move_step)
                        self.get_logger().info(f"Move right {move_step}cm")
                    else:
                        self.tello.move_left(move_step)
                        self.get_logger().info(f"Move left {move_step}cm")
                    moved = True
                if abs(dy) > tol:
                    if dy > 0:
                        self.tello.move_back(move_step)
                        self.get_logger().info(f"Move back {move_step}cm")
                    else:
                        self.tello.move_forward(move_step)
                        self.get_logger().info(f"Move forward {move_step}cm")
                    moved = True
                if not moved:
                    self.get_logger().info("AprilTag ID=0 aligned, preparing to land...")
                    break
            else:
                self.get_logger().info("No AprilTag ID=0 detected, trying to adjust or wait")
            time.sleep(1)
        self.tello.streamoff()
        self.tello.land()
            
    def keep_alive_callback(self):
        if not hasattr(self, 'tello'):
            try:
                self.connect_to_tello()
            except Exception as e:
                self.get_logger().error(f'Failed to connect to Tello: {e}')
                self.get_logger().info('After 10 seconds, trying to reconnect...')
                time.sleep(10)
                self.connect_to_tello()
        else:
            try:
                self.get_logger().info('Keeping Tello connection alive...')
                self.tello.send_command_without_return('command')
                battery_percentage = self.tello.get_battery()
                self.get_logger().info(f'Tello battery: {battery_percentage}%')
            except Exception as e:
                self.get_logger().error(f'Error while keeping Tello connection alive: {e}')
                self.get_logger().info('Trying to reconnect to Tello...')
                self.tello.end()
                del self.tello
    
    def connect_to_tello(self):
        self.tello = Tello()
        self.tello.connect()
        self.tello.send_command_without_return('command')
        self.get_logger().info('Tello connected successfully.')
        self.get_logger().info(f'The battery level is {self.tello.get_battery()}%')
        
        # self.tello.streamon()
        # self.get_logger().info('Tello video stream started.')
        # time.sleep(2)

    def reset_delivery_ready_callback(self, request, response):
        """
        Service callback to reset the delivery_ready flag.
        This allows the system to be ready for a new delivery after the previous one is completed.
        """
        self.delivery_ready = request.data
        response.success = True
        response.message = f'delivery_ready set to {self.delivery_ready}'
        self.get_logger().info(response.message)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = AutoDelivery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(node.get_name() + ' has been stopped by user.')
    finally:
        if hasattr(node, 'tello'):
            node.tello.end()
        node.get_logger().info('Tello connection closed.')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# End of file: uoa_robot_drone/uoa_robot_drone/auto_delivery.py

