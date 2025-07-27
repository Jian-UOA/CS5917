from djitellopy import Tello
import time
import asyncio

tello = Tello(host='192.168.8.21', retry_count=10)
tello.connect()
Tello.LOGGER.debug(f"Tello's battery: {tello.get_battery()}%")

# tello.connect_to_wifi(ssid="CS5917", password="abdn1234")

def delivery():
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    time.sleep(5)
    mid_from = tello.get_state_field('mid')
    print(f'Step1: Current MID: {mid_from}.')
    tello.disable_mission_pads()

    tello.takeoff()

    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    time.sleep(5)
    mid_from = tello.get_state_field('mid')
    print(f'Step2: Current MID: {mid_from}. After takeoff.')
    tello.disable_mission_pads()
    
    tello.connect()

    tello.move_forward(70)
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    time.sleep(5)
    mid_from = tello.get_state_field('mid')
    print(f'Step3: Current MID: {mid_from}. After takeoff.')
    tello.disable_mission_pads()
    
    time.sleep(3)
    tello.rotate_clockwise(180)
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    time.sleep(5)
    mid_from = tello.get_state_field('mid')
    print(f'Step4: Current MID: {mid_from}. After takeoff.')
    tello.disable_mission_pads()

    time.sleep(2)
    tello.land()
    Tello.LOGGER.debug('Delivery completed.')
    time.sleep(5)

    Tello.LOGGER.debug('Preparing to return to base...')
    tello.takeoff()
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    time.sleep(5)
    cmd = 'jump {} {} {} {} {} m{} m{}'.format(0, 0, 40, 40, 0, mid_from, mid_from)
    Tello.LOGGER.debug('Starting return to base...')
    res = tello.send_control_command(cmd)
    if not res:
        tello.move_forward(70)
        tello.enable_mission_pads()
        tello.set_mission_pad_detection_direction(2)
        time.sleep(5)
        cmd = 'jump {} {} {} {} {} m{} m{}'.format(0, 0, 40, 40, 0, mid_from, mid_from)
    time.sleep(2)
    tello.land()

def delivery():
    print("Starting delivery...")
    print(f'Tello\'s battery: {tello.get_battery()}%')
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    tello.takeoff()
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    time.sleep(5)
    while not tello.send_control_command('forward 70'):
        tello.send_control_command('forward 70')
    time.sleep(3)
    while not tello.send_control_command('rotate_clockwise 180'):
        tello.send_control_command('rotate_clockwise 180')
    time.sleep(3)
    while not tello.send_control_command('land'):
        tello.send_control_command('land')
    time.sleep(5)
    
    print("Starting return to base...")
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(2)
    while not tello.send_control_command('takeoff'):
        tello.send_control_command('takeoff')
    time.sleep(5)
    mid_from = tello.get_state_field('mid')
    if mid_from  < 1:
        while not tello.send_control_command('forward 70'):
            tello.send_control_command('forward 70')
        mid_from = tello.get_state_field('mid')
        print(f"Detected MID: {mid_from}")
        while mid_from < 1:
            tello.rotate_clockwise(15)
            time.sleep(0.5)
            yaw = tello.get_yaw()
            mid_from = tello.get_state_field('mid')
            print(f"Detected MID: {mid_from} at yaw {yaw}")
            if mid_from >= 1:
                print(f"Mission Pad {mid_from} detected. Proceeding with delivery.")
                break
    else:
        while not tello.send_control_command(f"go 0 0 100 30 m{mid_from}"):
            tello.send_control_command(f"go 0 0 100 30 m{mid_from}")
    
    mid_from = tello.get_state_field('mid')
    if mid_from  < 1:
        while not tello.send_control_command('rotate_clockwise 180'):
            tello.send_control_command('rotate_clockwise 180')
     

    if mid !=0:
        time.sleep(5)
        tello.go_xyz_speed_mid(0, 0, 80, 20, mid)
        yaw = tello.get_yaw()
        print(f"Yaw: {yaw}")
        time.sleep(1)
        yaw = tello.rotate_clockwise(-yaw)
        print(f"Yaw: {yaw}")
        tello.rotate_clockwise(-yaw)
        print(f"Yaw: {yaw}")
        time.sleep(2)
        tello.land()
    tello.disable_mission_pads()
    tello.end()

def main():
    delivery()
    # try:
    #     delivery()
    # except Exception as e:
    #     Tello.LOGGER.error(f'An error occurred during delivery: {e}')
    # finally:
    #     tello.end()
    #     Tello.LOGGER.info('Tello connection closed.')

if __name__ == '__main__':
    main()
# This code is intended to be run as a standalone script for testing purposes.