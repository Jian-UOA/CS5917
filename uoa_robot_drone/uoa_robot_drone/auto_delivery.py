"""
uoa_robot_drone - A ROS2 package for controlling a Tello drone for automated delivery

Testing commands:
- `ros2 launch uoa_robot_drone auto_delivery.launch.py`
- `ros2 service call /uoa/auto_delivery/reset_delivery_ready std_srvs/srv/SetBool "{data: true}"`
- `ros2 topic pub /uoa/delivery_signal std_msgs/Bool "{data: true}"`
- `ros2 topic pub /uoa/delivery_signal std_msgs/Bool "{data: false}"`

"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from djitellopy import Tello
import time
from std_srvs.srv import SetBool
import asyncio
import speech_recognition as sr
from .voice_controler import speak, listen_for_land_command

class AutoDelivery(Node):
    def __init__(self):
        super().__init__('auto_delivery')
        # Declare parameters
        self.declare_parameter('tello_ip', '192.168.12.20')
        self.declare_parameter('delivery_ready', False)
        # Additional parameters for voice control
        self.declare_parameter('prepare_takeoff_text', "Aberdeen No. 1 delivery drone will take off in 5 seconds to deliver the package. Please stay safe!")
        self.declare_parameter('customer_greeting', "Dear customer, Aberdeen No. 1 is delivering your express, please check it!")
        self.declare_parameter('prepare_return_text', "The package has been delivered. Aberdeen No. 1 will take off and return in 5 seconds. Please stay safe!")
        self.declare_parameter('land_ready_text', "Aberdeen number one express drone has completed delivery and is hovering, waiting for the land command.")
        self.declare_parameter('land_command_text', "number one can land")
        self.declare_parameter('land_starting_text', "Number one express drone is landing now.")
        self.declare_parameter('land_complete_text', "This delivery service is complete. Have a nice day! Goodbye!")

        # Get parameters
        self.tello_ip = self.get_parameter('tello_ip').get_parameter_value().string_value
        self.delivery_ready = self.get_parameter('delivery_ready').get_parameter_value().bool_value
        self.get_logger().info(f'delivery_ready: {self.delivery_ready}')
        self.land_ready_text = self.get_parameter('land_ready_text').get_parameter_value().string_value
        self.land_command_text = self.get_parameter('land_command_text').get_parameter_value().string_value
        self.land_starting_text = self.get_parameter('land_starting_text').get_parameter_value().string_value
        self.land_complete_text = self.get_parameter('land_complete_text').get_parameter_value().string_value

        # 设置本节点日志级别为DEBUG，便于后续通过参数或代码切换日志输出
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.connect_to_tello()
        self.keep_alive_timer = self.create_timer(10.0, self.keep_alive_callback)
        self.delivery_subscriber = self.create_subscription(
            Bool,
            'delivery_signal',
            self.delivery_callback,
            10
        )
        self.srv = self.create_service(SetBool, 'reset_delivery_ready', self.reset_delivery_ready_callback)

    def delivery_callback(self, msg):
        if not self.delivery_ready:
            self.get_logger().info('Delivery not ready, waiting for reset...')
            return
        if not msg.data:
            self.get_logger().info('No delivery signal received, waiting for delivery signal...')
            return
        self.get_logger().debug(f'Received delivery signal: {msg.data}')
        if not hasattr(self, 'tello'):
            try:
                self.connect_to_tello()
            except Exception as e:
                self.get_logger().error(f'Failed to connect to Tello: {e}')
                return
        try:
            if msg.data:
                # self.get_logger().info('Starting delivery...')
                asyncio.run(speak(self.get_parameter('prepare_takeoff_text').get_parameter_value().string_value))
                asyncio.run(speak("5"))
                asyncio.run(speak("4"))
                asyncio.run(speak("3"))
                asyncio.run(speak("2"))
                asyncio.run(speak("1"))
                asyncio.run(speak("Takeoff!"))
                self.tello.takeoff()
                time.sleep(4)
                self.tello.send_command_without_return()
                self.tello.move_forward(80)
                time.sleep(3)
                self.tello.send_command_without_return()
                self.tello.rotate_clockwise(180)
                asyncio.run(speak(self.get_parameter('customer_greeting').get_parameter_value().string_value))
                # time.sleep(2)
                self.tello.send_command_without_return()
                self.tello.land()
                self.get_logger().info('Delivery completed.')
                self.delivery_ready = False
                self.get_logger().debug(f'delivery_ready set to {self.delivery_ready}, waiting for reset...')
                time.sleep(2)
                asyncio.run(speak(self.get_parameter('prepare_return_text').get_parameter_value().string_value))
                # time.sleep(2)
                asyncio.run(speak("5"))
                asyncio.run(speak("4"))
                asyncio.run(speak("3"))
                asyncio.run(speak("2"))
                asyncio.run(speak("1"))
                asyncio.run(speak("Takeoff!"))
                self.tello.takeoff()
                self.get_logger().info('Starting return to base...')
                time.sleep(5)
                self.tello.move_forward(70)
                time.sleep(2)
                self.tello.rotate_clockwise(180)
                time.sleep(2)
                self.tello.move_down(80) 
                asyncio.run(speak(self.land_ready_text))
                listen_for_land_command(self.land_command_text, on_land_callback=self.on_land_callback)
                
            else:
                self.get_logger().info('No delivery signal received, waiting for delivery signal...')
        except Exception as e:
            self.get_logger().error(f'Error during delivery: {e}')
            self.tello.end()
            del self.tello  # Attempt to reset the Tello object
            self.get_logger().debug('Tello object reset, waiting for next delivery signal.')
        finally:
            self.get_logger().debug('Delivery callback finished, waiting for next signal...')
            # Ensure the Tello connection is properly closed after each delivery
            if hasattr(self, 'tello'):
                try:
                    self.tello.end()
                    del self.tello  # Attempt to reset the Tello object
                    self.get_logger().debug('Tello connection ended successfully.')
                except Exception as e:
                    self.get_logger().error(f'Error ending Tello connection: {e}')
            else:
                self.get_logger().debug('No Tello object to end, waiting for next delivery signal...')

    def on_land_callback(self):
        asyncio.run(speak(self.land_starting_text))
        self.tello.land()
        asyncio.run(speak(self.land_complete_text))

    def connect_to_tello(self):
        self.get_logger().debug('[connect_to_tello] Start')
        if hasattr(self, 'tello'):
            try:
                self.get_logger().debug('[connect_to_tello] Ending previous Tello connection...')
                self.tello.end()
                self.get_logger().debug('[connect_to_tello] Previous Tello connection ended.')
                time.sleep(2)  # Ensure the connection is properly closed
            except Exception as e:
                self.get_logger().error(f'[connect_to_tello] Error while ending previous Tello connection: {e}')
        try:
            self.get_logger().debug('[connect_to_tello] Instantiating Tello object...')
            # Use the Tello IP from parameters
            if not self.tello_ip:
                self.tello = Tello()
            else:
                self.tello = Tello(self.tello_ip)
            self.get_logger().debug('[connect_to_tello] Tello object instantiated.')
        except Exception as e:
            self.get_logger().error(f'[connect_to_tello] Error instantiating Tello: {e}')
            raise
        try:
            self.get_logger().debug('[connect_to_tello] Connecting to Tello...')
            self.tello.connect()
            self.get_logger().debug('[connect_to_tello] Tello.connect() success.')
        except Exception as e:
            self.get_logger().error(f'[connect_to_tello] Error in Tello.connect(): {e}')
            raise
        try:
            self.get_logger().debug('[connect_to_tello] Sending command mode...')
            self.tello.send_command_without_return('command')
            self.get_logger().debug('[connect_to_tello] Command mode sent.')
        except Exception as e:
            self.get_logger().error(f'[connect_to_tello] Error sending command mode: {e}')
            raise
        try:
            battery = self.tello.get_battery()
            self.get_logger().debug(f'[connect_to_tello] The battery level is {battery}%')
        except Exception as e:
            self.get_logger().error(f'[connect_to_tello] Error getting battery: {e}')
            raise
        self.get_logger().debug('[connect_to_tello] Finished')

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
                self.get_logger().debug('Waiting for delivery signal...')
                self.tello.send_command_without_return('command')
                battery_percentage = self.tello.get_battery()
                self.get_logger().debug(f'Tello battery: {battery_percentage}%')
            except Exception as e:
                self.get_logger().error(f'Error while keeping Tello connection alive: {e}')
                self.get_logger().debug('Trying to reconnect to Tello...')
                self.tello.end()
                del self.tello

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

