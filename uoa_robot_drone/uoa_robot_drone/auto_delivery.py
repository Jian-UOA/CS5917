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
import re

class AutoDelivery(Node):
    def __init__(self):
        super().__init__('auto_delivery')
        # Declare parameters
        self.declare_parameter('tello_ip', '192.168.183.21')
        self.declare_parameter('delivery_ready', False)
        # Additional parameters for voice control
        self.declare_parameter('prepare_takeoff_text', "Received delivery order. Aberdeen No. 1 delivery drone will take off in 5 seconds to deliver the package. Please stay safe!")
        self.declare_parameter('customer_greeting_text', "Dear customer, Aberdeen No. 1 is delivering your express and then I will land for you to pick up the package. Please stay safe!")
        self.declare_parameter('prepare_return_text', "The package has been delivered. Aberdeen No. 1 will take off and return in 5 seconds. Please stay safe!")
        self.declare_parameter('customer_goodbye_text', "Dear customer, thank you for using Aberdeen No. 1 delivery service. Have a wonderful day! Goodbye!")
        self.declare_parameter('seeking_base_vehicle_text', "Aberdeen number one express drone is seeking the base vehicle.")
        self.declare_parameter('land_ready_text', "Aberdeen number one express drone has completed delivery and is hovering, waiting for the land command.")
        self.declare_parameter('land_command_text', "number one can land")
        self.declare_parameter('land_starting_text', "Number one express drone is landing now.")

        self.declare_parameter('land_complete_text', "This delivery service is complete. Have a nice day! Goodbye!")

        # Get parameters
        self.tello_ip = self.get_parameter('tello_ip').get_parameter_value().string_value
        self.delivery_ready = self.get_parameter('delivery_ready').get_parameter_value().bool_value
        self.get_logger().info(f'delivery_ready: {self.delivery_ready}')
        self.prepare_takeoff_text = self.get_parameter('prepare_takeoff_text').get_parameter_value().string_value
        self.customer_greeting_text = self.get_parameter('customer_greeting_text').get_parameter_value().string_value
        self.prepare_return_text = self.get_parameter('prepare_return_text').get_parameter_value().string_value
        self.customer_goodbye_text = self.get_parameter('customer_goodbye_text').get_parameter_value().string_value
        self.seeking_base_vehicle_text = self.get_parameter('seeking_base_vehicle_text').get_parameter_value().string_value
        self.land_ready_text = self.get_parameter('land_ready_text').get_parameter_value().string_value
        self.land_command_text = self.get_parameter('land_command_text').get_parameter_value().string_value
        self.land_starting_text = self.get_parameter('land_starting_text').get_parameter_value().string_value
        self.land_complete_text = self.get_parameter('land_complete_text').get_parameter_value().string_value

        # Set the logging level of this node to DEBUG for development and debugging purposes
        # This should be set to INFO or WARN in production
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.connect_to_tello()
        self.keep_alive_timer = self.create_timer(10.0, self.keep_alive)

        # Create a flag to track the current delivery signal state for the keep_alive method
        # Its value will be set to True and False respectively when a delivery signal is received or processed in the delivery_callback method
        self.current_delivery_signal = Bool()
        self.current_delivery_signal.data = False

        self.delivery_subscriber = self.create_subscription(
            Bool,
            'delivery_signal',
            self.delivery_callback,
            10
        )

        self.srv = self.create_service(SetBool, 'reset_delivery_ready', self.reset_delivery_ready_callback)

    def connect_to_tello(self):
        self.get_logger().debug('[connect_to_tello] Start')
        while True:
            try:
                self.get_logger().debug('[connect_to_tello] Instantiating Tello object...')
                if not hasattr(self, 'tello'):
                    # Use the Tello IP from parameters
                    if not self.tello_ip:
                        self.tello = Tello()
                    else:
                        self.tello = Tello(self.tello_ip)
                    self.get_logger().debug('[connect_to_tello] Tello object instantiated.')
                
                self.tello.connect()
                self.get_logger().debug('[connect_to_tello] Tello.connect() success.')
                break  # Exit the loop if connection is successful
            except Exception as e:
                self.get_logger().error(f'[connect_to_tello] Error: {e}')
                time.sleep(5)
                continue
        
        try:
            battery = self.tello.get_battery()
            self.get_logger().info(f'[connect_to_tello] The battery level is {battery}%')
        except Exception as e:
            self.get_logger().error(f'[connect_to_tello] Error getting battery: {e}')

    def keep_alive(self):
        """Send a keepalive packet to prevent the drone from landing after 15 seconds."""
        try:
            battery = self.tello.get_battery()
            self.get_logger().info(f'[keep_alive] The battery level is {battery}%')

            if 

            if not self.current_delivery_signal.data:
                self.get_logger().info('[keep_alive] Waiting for delivery signal...')
        except Exception as e:
            self.get_logger().error(f'[keep_alive] Error sending keepalive: {e}')
            # Attempt to reconnect if keepalive fails
            self.connect_to_tello()

    def delivery_callback(self, msg):
        if not self.delivery_ready:
            self.get_logger().info('Delivery not ready, waiting for reset...')
            return
        
        if not msg.data:
            self.get_logger().info('No delivery order received, waiting for delivery signal...')
            return
        else:
            self.current_delivery_signal.data = msg.data
            self.get_logger().info(f'Received delivery signal: {msg.data}, starting delivery process...')

        if not hasattr(self, 'tello'):
            try:
                self.connect_to_tello()
            except Exception as e:
                self.get_logger().error(f'Failed to connect to Tello: {e}')
                return
        try:
            if msg.data:
                # self.get_logger().info('Starting delivery...')
                self.get_logger().debug(self.prepare_takeoff_text)
                asyncio.run(speak(self.get_parameter('prepare_takeoff_text').get_parameter_value().string_value))
                asyncio.run(speak("5"))
                asyncio.run(speak("4"))
                asyncio.run(speak("3"))
                asyncio.run(speak("2"))
                asyncio.run(speak("1"))
                asyncio.run(speak("Takeoff!"))
                self.tello.enable_mission_pads()
                self.tello.set_mission_pad_detection_direction(2)
                self.tello.takeoff()
                time.sleep(5)
               
                mid_from = self.tello.get_state_field('mid')
                self.get_logger().info(f"Detected MID: {mid_from}")

                self.tello.move_up(25)
                time.sleep(2)
                self.tello.move_forward(80)
                time.sleep(5)
                
                self.tello.rotate_clockwise(180)

                self.get_logger().debug(self.customer_greeting_text)
                asyncio.run(speak(self.customer_greeting_text))
                self.tello.land()
                time.sleep(5)
                self.get_logger().info('Delivery completed.')
                self.delivery_ready = False
                self.current_delivery_signal.data = False
                self.get_logger().debug(f'delivery_ready set to {self.delivery_ready}, waiting for reset...')
                
                # time.sleep(2)
                self.get_logger().debug(self.prepare_return_text)
                asyncio.run(speak(self.prepare_return_text))
                
                # time.sleep(2)
                asyncio.run(speak("5"))
                asyncio.run(speak("4"))
                asyncio.run(speak("3"))
                asyncio.run(speak("2"))
                asyncio.run(speak("1"))
                asyncio.run(speak("Takeoff!"))

                self.tello.enable_mission_pads()
                self.tello.set_mission_pad_detection_direction(2)
                self.tello.takeoff()

                mid_from = self.tello.get_state_field('mid')
                self.get_logger().info(f"Detected MID: {mid_from} at customer's location")
                
                self.get_logger().debug(self.customer_goodbye_text)
                asyncio.run(speak(self.customer_goodbye_text))
                
                # if mid_from < 1:
                #     mid_from = self.tello.get_state_field('mid')
                #     self.get_logger().info(f"Second time detected MID: {mid_from} at customer's location")
                
                if mid_from >= 1:
                    self.get_logger().info('Starting return to base vehicle...')
                    cmd = f'go 0 0 110 20 m{mid_from}'
                    while not self.tello.send_control_command(cmd):
                        time.sleep(8)
                        self.tello.send_control_command(cmd)
                        break
                    self.pose_rectify()
                else:
                    self.get_logger().info('No MID detected, moving forward to search for MID...')
                    while not self.tello.send_control_command('forward 80'):
                        time.sleep(8)
                        self.tello.send_control_command('forward 80')
                    self.tello.enable_mission_pads()
                    self.tello.set_mission_pad_detection_direction(2)
                    time.sleep(5)
                    mid_from = self.tello.get_state_field('mid')
                    self.get_logger().info(f"Detected MID: {mid_from} after moving forward")
                    if mid_from < 1:
                        self.get_logger().info('No MID detected, rotating to search for MID...')
                        while mid_from < 1:
                            asyncio.run(speak(self.seeking_base_vehicle_text))
                            self.tello.rotate_clockwise(15)
                            time.sleep(0.5)
                            yaw = self.tello.get_yaw()
                            mid_from = self.tello.get_state_field('mid')
                            self.get_logger().info(f"Detected MID: {mid_from} at yaw {yaw}")
                            if mid_from >= 1:
                                self.get_logger().info(f'Detected Base Vehicle ID: {mid_from}. Preparing for landing.')
                                break
                        self.pose_rectify()
                self.get_logger().debug(self.land_ready_text)
                asyncio.run(speak(self.land_ready_text))
                listen_for_land_command(on_land_callback=self.voice_control_callback)     
        except Exception as e:
            self.get_logger().error(f'Error during delivery: {e}')

    def pose_rectify(self):
        """
        Rectify the pose of the Tello drone.
        This function is a placeholder for any pose rectification logic needed.
        """
        self.get_logger().info('Rectifying pose...')

        yaw = self.tello.get_yaw()
        self.get_logger().info(f'Current yaw: {yaw}')
        while not self.tello.send_control_command(f'ccw {yaw}'):
            time.sleep(8)
            self.tello.send_control_command(f'ccw {yaw}')
        time.sleep(8)
        yaw = self.tello.get_yaw()
        self.get_logger().info(f'Current yaw: {yaw}')
        while not self.tello.send_control_command(f'ccw {yaw}'):
            time.sleep(8)
            self.tello.send_control_command(f'ccw {yaw}')

        self.get_logger().info('Pose rectified.')

    def voice_control_callback(self, order_text):
        self.get_logger().info(f'Voice command received: {order_text}')
        direction, value = self.parse_order_text(order_text)
        if direction and value:
            if direction == 'land':
                while not self.tello.send_control_command('land'):
                    time.sleep(8)
                    self.tello.send_control_command('land')
                asyncio.run(speak(self.land_complete_text))
                return True  # Indicate that the land command was executed
            elif value < 20 or value > 500:
                asyncio.run(speak("Sorry, the unit is centimeters and the value must be greater than 20 and less than 500."))
                return False  # Indicate that the command was not executed
            else:
                while not self.tello.send_control_command(f'{direction} {value}'):
                    time.sleep(1)
                    self.tello.send_control_command(f'{direction} {value}')
                return False  # Indicate that the command was executed but not a land command

    def parse_order_text(self, order_text):
        pattern = r'^(up|down|left|right|forward|back)\s+(\d+)$'
        match = re.match(pattern, order_text.strip(), re.IGNORECASE)
        if match:
            direction = match.group(1).lower()
            value = int(match.group(2))
            return direction, value
        elif 'land' in order_text.lower():
            return 'land', 0
        else:
            return None, None

    def on_land_callback(self):
        asyncio.run(speak(self.land_starting_text))
        response = self.tello.send_control_command('mon')
        while response != 'ok':
            self.get_logger().debug(f'Waiting for Tello to respond to land command, current response: {response}')
            time.sleep(1)
            response = self.tello.send_control_command('mon')
        self.get_logger().debug('Tello is ready to land.')
        time.sleep(2)  # Wait for Tello to stabilize before landing
        self.get_logger().info('Landing Tello...')
        asyncio.run(speak("Landing now!"))
        
        self.tello.land()
        asyncio.run(speak(self.land_complete_text))
    
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

