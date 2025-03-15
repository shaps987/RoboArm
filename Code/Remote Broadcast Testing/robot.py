#!/home/pi/myenv/bin/python3

# --- Import Required Libraries ---
import numpy as np
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
from bleak import BleakScanner, BleakClient
import asyncio
# import machine
import RPi.GPIO as GPIO

# --- Robot Arm Constants ---
L1 = 6
L2 = 6
L3 = 2
X_MIN = 0
Y_MIN = 0

# --- GPIO and I2C Setup ---
GPIO.setmode(GPIO.BCM)
# i2c = busio.I2C(SCL, SDA)
# pca = PCA9685(i2c)
# pca.frequency = 50  # Servo Frequency (50 Hz)

# --- Built-In LED Setup ---
# led = machine.Pin("LED", machine.Pin.OUT)

# --- Global Variables ---
connected = False
alive = False

# --- Define Servo Channels and Set Pulse Ranges --- 
# joint1 = pca.channels[0]  # Servo on channel 0
# joint2 = pca.channels[1]  # Servo on channel 1
# joint3 = pca.channels[2]  # Servo on channel 2
# joint_claw = pca.channels[3]  # Servo on channel 3

# --- Initialize Angle Tracking ---
# servo_angles = {0: 0, 1: 0, 2: 0}  # Track angles for channels 0, 1, and 2

# --- Create move_servo Function ---
# def move_servo(channel, angle):
#     """
#     Move a servo connected to the PCA9685 to a specific angle and track the angle.
    
#     Parameters:
#         channel (int): The PCA9685 channel the servo is connected to (0-15).
#         angle (float): Desired servo angle in degrees (0-180).
#     """
#     # Validate input
#     if channel not in servo_angles:
#         raise ValueError(f"Channel {channel} is not being tracked. Only channels 0, 1, and 2 are tracked.")

#     if angle < 0 or angle > 180:
#         raise ValueError("Angle must be between 0 and 180 degrees.")

#     # MG996R pulse width range (in microseconds)
#     min_pulse = 500  # Corresponds to 0 degrees
#     max_pulse = 2500  # Corresponds to 180 degrees

#     # Map angle (0–180) to pulse width
#     pulse_width = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))

#     # Convert pulse width to duty cycle
#     duty_cycle = int((pulse_width / 1000000) * pca.frequency * 65536)

#     # Set the duty cycle for the specified channel
#     pca.channels[channel].duty_cycle = duty_cycle

#     # Update the tracked angle for the channel
#     servo_angles[channel] = angle
#     print(f"Servo on channel {channel} moved to {angle} degrees.")

# --- Create get_servo_angle Function ---
# def get_servo_angle(channel):
    # """
    # Retrieve the current angle of a tracked servo.
    
    # Parameters:
    #     channel (int): The PCA9685 channel of the servo (0-2).
    
    # Returns:
    #     float: The current angle of the servo.
    # """
    # if channel not in servo_angles:
    #     raise ValueError(f"Channel {channel} is not being tracked. Only channels 0, 1, and 2 are tracked.")

    # return servo_angles[channel]

# --- Calculate Claw Position ---
# def calculate_claw_position(l1, l2, l3):
    # """
    # Calculate the x and y position of the claw (endpoint) using current servo angles.

    # Parameters:
    #     l1 (float): Length of the first arm segment.
    #     l2 (float): Length of the second arm segment.
    #     l3 (float): Length of the third arm segment.

    # Returns:
    #     tuple: (x, y) coordinates of the claw (endpoint).
    # """
    # # Get the current angles of the servos
    # angle1 = get_servo_angle(0) - 90  # Convert to [-90, 90] range
    # angle2 = get_servo_angle(1) - 90
    # angle3 = get_servo_angle(2) - 90

    # # Convert angles to radians for trigonometric calculations
    # angle1_rad = np.radians(angle1)
    # angle2_rad = np.radians(angle2)
    # angle3_rad = np.radians(angle3)

    # # Calculate the (x, y) position of each joint
    # joint1_x = l1 * np.cos(angle1_rad)
    # joint1_y = l1 * np.sin(angle1_rad)

    # joint2_x = joint1_x + l2 * np.cos(angle1_rad + angle2_rad)
    # joint2_y = joint1_y + l2 * np.sin(angle1_rad + angle2_rad)

    # claw_x = joint2_x + l3 * np.cos(angle1_rad + angle2_rad + angle3_rad)
    # claw_y = joint2_y + l3 * np.sin(angle1_rad + angle2_rad + angle3_rad)

    # if claw_x < X_MIN and claw_y < Y_MIN:
    #     return claw_x, claw_y
    # else:
    #     return None

# --- Calculate Inverse Kinematics Servo Angles ---
# def compute_servo_angles(l1, l2, l3, x, y):
    # """
    # Computes the joint angles and servo angles for a robotic arm.
    
    # Parameters:
    #     l1 (float): Length of the first segment.
    #     l2 (float): Length of the second segment.
    #     l3 (float): Length of the third segment.
    #     x (float): Target x-coordinate of the endpoint.
    #     y (float): Target y-coordinate of the endpoint.
    
    # Returns:
    #     tuple: (joint_angles, servo_angles)
    #         - joint_angles: A tuple of (angle1, angle2, angle3) in degrees.
    #         - servo_angles: A tuple of (servo_angle1, servo_angle2, servo_angle3) in the range [0, 180].
    # """
    # def map_to_servo_range(angle):
    #     # Map the angle (in degrees) to the servo range [0, 180]
    #     return np.clip(angle + 90, 0, 180)

    # # Adjust the target to account for the horizontal third segment
    # x_adj = x - l3  # Adjust for the third segment
    # y_adj = y

    # # Calculate the distance to the adjusted target
    # r_adj = np.sqrt(x_adj**2 + y_adj**2)

    # # Constrain the target to be within reach of the first two segments
    # if r_adj > l1 + l2:
    #     scale = (l1 + l2) / r_adj
    #     x_adj *= scale
    #     y_adj *= scale

    # # Calculate angles using the Law of Cosines
    # cos_angle2 = (r_adj**2 - l1**2 - l2**2) / (2 * l1 * l2)
    # angle2 = np.arccos(np.clip(cos_angle2, -1, 1))  # Angle at Joint 2

    # k1 = l1 + l2 * np.cos(angle2)
    # k2 = l2 * np.sin(angle2)
    # angle1 = np.arctan2(y_adj, x_adj) - np.arctan2(k2, k1)

    # # Calculate the angle of the line between (0, 0) and the adjusted joint2
    # joint1_x, joint1_y = l1 * np.cos(angle1), l1 * np.sin(angle1)
    # joint2_x = joint1_x + l2 * np.cos(angle1 + angle2)
    # joint2_y = joint1_y + l2 * np.sin(angle1 + angle2)
    # line_angle = np.arctan2(joint2_y, joint2_x)

    # # Mirror the first and second segments along the imaginary line
    # angle1 = 2 * line_angle - angle1
    # angle2 = -angle2

    # # Calculate the third segment's angle to make it horizontal
    # angle3 = - (angle1 + angle2)

    # # Convert angles to degrees
    # angle1_deg = np.degrees(angle1)
    # angle2_deg = np.degrees(angle2)
    # angle3_deg = np.degrees(angle3)

    # # Map the angles to the servo range [0, 180]
    # servo_angle1 = map_to_servo_range(angle1_deg)
    # servo_angle2 = map_to_servo_range(angle1_deg + angle2_deg)  # Relative adjustment for servo 2
    # servo_angle3 = map_to_servo_range(0)  # Third segment always horizontal

    # # joint_angles = (angle1_deg, angle2_deg, angle3_deg)
    # servo_angles = (servo_angle1, servo_angle2, servo_angle3)

    # return servo_angles

# --- BLE Constants ---
REMOTE_NAME = "RoboticArmRemote"
REMOTE_UUID = "1848"
COMMAND_UUID = "2A6E"

# --- BLE Remote Finder ---
async def find_remote():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name == REMOTE_NAME:
            print(f"Found remote: {device}")
            return device
    print("Remote not found.")
    return None

# --- Notification Handler ---
def notification_handler(sender, data):
    """
    Handle incoming BLE notifications.
    
    Parameters:
        sender: The source of the notification.
        data: The data received in the notification.
    """
    print(f"Notification received from {sender}: {data}")
    move_robot(data)  # Pass the data to the move_robot function

# --- BLE Connection Task ---
async def connect_to_remote():
    global connected, alive
    retries = 0
    while not connected and retries < 5:  # Retry up to 5 times
        device = await find_remote()
        if not device:
            print("No remote found. Retrying...")
            retries += 1
            await asyncio.sleep(2)
            continue
        try:
            print(f"Connecting to {device}...")
            async with BleakClient(device) as client:
                connected = True
                alive = True
                print(f"Connected to remote: {device.name}")
                
                await client.start_notify(COMMAND_UUID, notification_handler)
                print("Subscribed to notifications.")
                while connected:
                    await asyncio.sleep(1)
        except Exception as e:
            print(f"Connection error: {e}")
            retries += 1
        finally:
            connected = False
            alive = False
            print("Disconnected.")

# # --- Motor Controller Setup ---
# # Rear Motors (in1 and in2 are for the right motor and in3 and in4 are for the left motor)
# in1_rear, in2_rear, en_a_rear = 5, 6, 12
# GPIO.setup(in1_rear, GPIO.OUT)
# GPIO.setup(in2_rear, GPIO.OUT)
# GPIO.setup(en_a_rear, GPIO.OUT)
# rear_right = GPIO.PWM(en_a_rear, 100)
# rear_right.start(75)

# in3_rear, in4_rear, en_b_rear = 19, 26, 13
# GPIO.setup(in3_rear, GPIO.OUT)
# GPIO.setup(in4_rear, GPIO.OUT)
# GPIO.setup(en_b_rear, GPIO.OUT)
# rear_left = GPIO.PWM(en_b_rear, 100)
# rear_left.start(75)

# # Front Motors (in1 and in2 are for the right motor and in3 and in4 are for the left motor)
# in1_front, in2_front, en_a_front = 23, 24, 18
# GPIO.setup(in1_front, GPIO.OUT)
# GPIO.setup(in2_front, GPIO.OUT)
# GPIO.setup(en_a_front, GPIO.OUT)
# front_right = GPIO.PWM(en_a_front, 100)
# front_right.start(75)

# in3_front, in4_front, en_b_front = 27, 22, 25
# GPIO.setup(in3_front, GPIO.OUT)
# GPIO.setup(in4_front, GPIO.OUT)
# GPIO.setup(en_b_front, GPIO.OUT)
# front_left = GPIO.PWM(en_b_front, 100)
# front_left.start(75)

# --- Robot Movement Function ---
def move_robot(command):
    """ Execute robot movement based on the received command """
    print(f"Processing command: {command}")
    if command == b'f':  # Forward
        print("Forward")

    elif command == b'b':  # Backward
        print("Backward")

    elif command == b'r':  # Strafe Right
        print("Strafe Right")

    elif command == b'l':  # Strafe Left
        print("Strafe Left")

    elif command == b'q':  # Turn Right
        print("Turn Right")

    elif command == b'p':  # Turn Left
        print("Turn Left")

    elif command == b'a':  # Arm Forward
        print("Arm Forward")

    elif command == b'c':  # Arm Backward
        print("Arm Backward")

    elif command == b'y':  # Claw Open
        print("Claw Open")

    elif command == b'z':  # Claw Closed
        print("Claw Closed")

    elif command == b'd':  # Arm Up
        print("Arm Up")

    elif command == b'e':  # Arm Down
        print("Arm Down")

    else:
        print(f"Unknown command: {command}")


# --- LED Blink Task ---
async def blink_task():
    pass
    # global alive
    # print("Blink task started.")
    # toggle = True
    # while alive:
    #     blink_duration = 250 if not connected else 1000
    #     led.value(toggle)
    #     toggle = not toggle
    #     await asyncio.sleep_ms(blink_duration)
    # print("Blink task stopped.")

# --- Main Task ---
async def main():
    tasks = [
        asyncio.create_task(blink_task()),
        asyncio.create_task(connect_to_remote()),
    ]
    await asyncio.gather(*tasks)

# --- Script Entry Point ---
asyncio.run(main())
