# --- Import Required Libraries ---
import math as math
from adafruit_pca9685 import PCA9685
import board
import busio
import time
import pwmio
from digitalio import DigitalInOut, Direction
from circuitpython_nrf24l01.rf24 import RF24

# --- Robot Arm Constants ---
L1 = 6
L2 = 6
L3 = 2
X_MIN = 0
Y_MIN = 0

# --- Initialize SPI and RX Logic --- #
spi = busio.SPI(board.GP18, board.GP19, board.GP16)
ce  = DigitalInOut(board.GP14);  ce.direction = Direction.OUTPUT
csn = DigitalInOut(board.GP15); csn.direction = Direction.OUTPUT

radio = RF24(spi, csn, ce)
radio.channel = 100            # choose 0–125
radio.payload_size = 1         # one-byte packets
radio.open_rx_pipe(1, b'\xe1\xf0\xf0\xf0\xf0')
print("RX: channel =", radio.channel,
      "payload_size =", radio.payload_size,
      "pipe1 addr =", b'\xe1\xf0\xf0\xf0\xf0')
radio.listen = True        # enable receive mode

# --- GPIO and I2C Setup ---
i2c = busio.I2C(board.GP1, board.GP0)
pca = PCA9685(i2c)
pca.frequency = 50  # Servo Frequency (50 Hz)

# --- Built-In LED Setup ---
# led = machine.Pin("LED", machine.Pin.OUT)

# --- Define Servo Channels and Set Pulse Ranges --- 
joint1 = pca.channels[0]  # Servo on channel 0
joint2 = pca.channels[1]  # Servo on channel 1
joint3 = pca.channels[2]  # Servo on channel 2
joint_claw = pca.channels[3]  # Servo on channel 3

# --- Initialize Angle Tracking ---
servo_angles = {0: 0, 1: 0, 2: 0, 3: 0}  # Track angles for channels 0, 1, and 2

# --- Create move_servo Function ---
def move_servo(channel, angle):
    """
    Move a servo connected to the PCA9685 to a specific angle and track the angle.
    
    Parameters:
        channel (int): The PCA9685 channel the servo is connected to (0-15).
        angle (float): Desired servo angle in degrees (0-180).
    """
    # Validate imathut
    if channel not in servo_angles:
        raise ValueError(f"Channel {channel} is not being tracked. Only channels {list(servo_angles)} are tracked.")

    if angle < 0 or angle > 180:
        raise ValueError("Angle must be between 0 and 180 degrees.")

    # MG996R pulse width range (in microseconds)
    min_pulse = 500  # Corresponds to 0 degrees
    max_pulse = 2500  # Corresponds to 180 degrees

    # Map angle (0–180) to pulse width
    pulse_width = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))

    # Convert pulse width to duty cycle
    duty_cycle = int((pulse_width / 1000000) * pca.frequency * 65536)

    # Set the duty cycle for the specified channel
    pca.channels[channel].duty_cycle = duty_cycle

    # Update the tracked angle for the channel
    servo_angles[channel] = angle
    print(f"Servo on channel {channel} moved to {angle} degrees.")

# --- Create get_servo_angle Function ---
def get_servo_angle(channel):
    """
    Retrieve the current angle of a tracked servo.
    
    Parameters:
        channel (int): The PCA9685 channel of the servo (0-3).
    
    Returns:
        float: The current angle of the servo.
    """
    if channel not in servo_angles:
        raise ValueError(f"Channel {channel} is not being tracked. Only channels {list(servo_angles)} are tracked.")
    return servo_angles[channel]

# --- Calculate Claw Position ---
def calculate_claw_position(l1, l2, l3):
    """
    Calculate the x and y position of the claw (endpoint) using current servo angles.

    Parameters:
        l1 (float): Length of the first arm segment.
        l2 (float): Length of the second arm segment.
        l3 (float): Length of the third arm segment.

    Returns:
        tuple: (x, y) coordinates of the claw (endpoint).
    """
    # Get the current angles of the servos
    angle1 = get_servo_angle(0) - 90  # Convert to [-90, 90] range
    angle2 = get_servo_angle(1) - 90
    angle3 = get_servo_angle(2) - 90

    # Convert angles to radians for trigonometric calculations
    angle1_rad = math.radians(angle1)
    angle2_rad = math.radians(angle2)
    angle3_rad = math.radians(angle3)

    # Calculate the (x, y) position of each joint
    joint1_x = l1 * math.cos(angle1_rad)
    joint1_y = l1 * math.sin(angle1_rad)

    joint2_x = joint1_x + l2 * math.cos(angle1_rad + angle2_rad)
    joint2_y = joint1_y + l2 * math.sin(angle1_rad + angle2_rad)

    claw_x = joint2_x + l3 * math.cos(angle1_rad + angle2_rad + angle3_rad)
    claw_y = joint2_y + l3 * math.sin(angle1_rad + angle2_rad + angle3_rad)

    if claw_x < X_MIN and claw_y < Y_MIN:
        return claw_x, claw_y
    else:
        return None

# --- Calculate Inverse Kinematics Servo Angles ---
def compute_servo_angles(l1, l2, l3, x, y):
    """
    Computes the joint angles and servo angles for a robotic arm.
    
    Parameters:
        l1 (float): Length of the first segment.
        l2 (float): Length of the second segment.
        l3 (float): Length of the third segment.
        x (float): Target x-coordinate of the endpoint.
        y (float): Target y-coordinate of the endpoint.
    
    Returns:
        tuple: (joint_angles, servo_angles)
            - joint_angles: A tuple of (angle1, angle2, angle3) in degrees.
            - servo_angles: A tuple of (servo_angle1, servo_angle2, servo_angle3) in the range [0, 180].
    """

    # Adjust the target to account for the horizontal third segment
    x_adj = x - l3  # Adjust for the third segment
    y_adj = y

    # Calculate the distance to the adjusted target
    r_adj = math.sqrt(x_adj**2 + y_adj**2)

    # Constrain the target to be within reach of the first two segments
    if r_adj > l1 + l2:
        scale = (l1 + l2) / r_adj
        x_adj *= scale
        y_adj *= scale

    # Calculate angles using the Law of Cosines
    cos_angle2 = (r_adj**2 - l1**2 - l2**2) / (2 * l1 * l2)
    angle2    = math.acos(max(-1, min(1, cos_angle2)))  # Angle at Joint 2

    k1 = l1 + l2 * math.cos(angle2)
    k2 = l2 * math.sin(angle2)
    angle1 = math.atan2(y_adj, x_adj) - math.atan2(k2, k1)

    # Calculate the angle of the line between (0, 0) and the adjusted joint2
    joint1_x, joint1_y = l1 * math.cos(angle1), l1 * math.sin(angle1)
    joint2_x = joint1_x + l2 * math.cos(angle1 + angle2)
    joint2_y = joint1_y + l2 * math.sin(angle1 + angle2)
    line_angle = math.atan2(joint2_y, joint2_x)

    # Mirror the first and second segments along the imaginary line
    angle1 = 2 * line_angle - angle1
    angle2 = -angle2

    # Calculate the third segment's angle to make it horizontal
    angle3 = - (angle1 + angle2)

    # Convert angles to degrees
    angle1_deg = math.degrees(angle1)
    angle2_deg = math.degrees(angle2)
    angle3_deg = math.degrees(angle3)

    # Map the angles to the servo range [0, 180]
    servo_angle1 = 90-angle1_deg
    servo_angle2 = angle2_deg+90  # Relative adjustment for servo 2
    servo_angle3 = -(angle3_deg)  # Horizontal correction

    # joint_angles = (angle1_deg, angle2_deg, angle3_deg)
    servo_angles = (servo_angle1, servo_angle2, servo_angle3)

    return servo_angles


# --- Motor Controller Setup ---
# --- L298N #1 (Controls 2 motors) ---
in1_1 = DigitalInOut(board.GP2)
in1_1.direction = Direction.OUTPUT

in2_1 = DigitalInOut(board.GP3)
in2_1.direction = Direction.OUTPUT

in3_1 = DigitalInOut(board.GP4)
in3_1.direction = Direction.OUTPUT

in4_1 = DigitalInOut(board.GP5)
in4_1.direction = Direction.OUTPUT

ena_1 = pwmio.PWMOut(board.GP6, frequency=100)
ena_1.duty_cycle = int(0.75 * 65535)   # 75% power

enb_1 = pwmio.PWMOut(board.GP7, frequency=100)
enb_1.duty_cycle = int(0.75 * 65535)   # 75% power


# --- L298N #2 (Controls 2 motors) ---
in1_2 = DigitalInOut(board.GP8)
in1_2.direction = Direction.OUTPUT

in2_2 = DigitalInOut(board.GP9)
in2_2.direction = Direction.OUTPUT

in3_2 = DigitalInOut(board.GP10)
in3_2.direction = Direction.OUTPUT

in4_2 = DigitalInOut(board.GP11)
in4_2.direction = Direction.OUTPUT

ena_2 = pwmio.PWMOut(board.GP12, frequency=100)
ena_2.duty_cycle = int(0.75 * 65535)   # 75% power

enb_2 = pwmio.PWMOut(board.GP13, frequency=100)
enb_2.duty_cycle = int(0.75 * 65535)   # 75% power

# --- Robot Movement Function ---
def move_robot(cmd: bytes):
    """ Execute robot movement based on the received command """
    print(f"Processing command: {cmd!r}")
    try:
        # --- Chassis (rear = bridge #1, front = bridge #2) ---
        if cmd == b'f':           # forward
            in1_1.value, in2_1.value, in3_1.value, in4_1.value = False, True, True, False
            in1_2.value, in2_2.value, in3_2.value, in4_2.value = False, True, True, False

        elif cmd == b'b':         # backward
            in1_1.value, in2_1.value, in3_1.value, in4_1.value = True, False, False, True
            in1_2.value, in2_2.value, in3_2.value, in4_2.value = True, False, False, True

        elif cmd == b'r':         # strafe right
            # RR ▶ forward, RL ▶ backward, FR ▶ backward, FL ▶ forward
            in1_1.value, in2_1.value = True, False
            in3_1.value, in4_1.value = True, False
            in1_2.value, in2_2.value = True, False
            in3_2.value, in4_2.value = True, False

        elif cmd == b'l':         # strafe left
            # RR ▶ backward, RL ▶ forward, FR ▶ forward,  FL ▶ backward
            in1_1.value, in2_1.value = False, True
            in3_1.value, in4_1.value = False, True
            in1_2.value, in2_2.value = False, True
            in3_2.value, in4_2.value = False, True

        elif cmd == b'q':  # turn right (in place)
            in1_1.value, in2_1.value, in3_1.value, in4_1.value = True, False, False, True
            in1_2.value, in2_2.value, in3_2.value, in4_2.value = False, True, True, False

        elif cmd == b'p':  # turn left (in place)
            in1_1.value, in2_1.value, in3_1.value, in4_1.value = False, True, True, False
            in1_2.value, in2_2.value, in3_2.value, in4_2.value = True, False, False, True

        elif cmd == b"s":   # STOP
            in1_1.value = in2_1.value = in3_1.value = in4_1.value = False
            in1_2.value = in2_2.value = in3_2.value = in4_2.value = False

        # --- Arm & Claw (same as before) ---
        elif cmd == b'a':  # Arm Forward
            claw_pos = calculate_claw_position(L1, L2, L3)
            angles   = compute_servo_angles(L1, L2, L3, claw_pos[0] + 1, claw_pos[1])
            if angles:
                move_servo(0, angles[0])
                move_servo(1, angles[1])
                move_servo(2, angles[2])

        elif cmd == b'c':  # Arm Backward
            claw_pos = calculate_claw_position(L1, L2, L3)
            angles   = compute_servo_angles(L1, L2, L3, claw_pos[0] - 1, claw_pos[1])
            if angles:
                move_servo(0, angles[0])
                move_servo(1, angles[1])
                move_servo(2, angles[2])

        elif cmd == b'y':  # Claw Open
            move_servo(3, 45)

        elif cmd == b'z':  # Claw Closed
            move_servo(3, 0)

        elif cmd == b'a':  # Arm Forward
            claw_pos = calculate_claw_position(L1, L2, L3)
            angles = compute_servo_angles(L1, L2, L3, claw_pos[0] + 1, claw_pos[1])
            if angles is not None:
                move_servo(0, angles[0])
                move_servo(1, angles[1])
                move_servo(2, angles[2])

        elif cmd == b'c':  # Arm Backward
            claw_pos = calculate_claw_position(L1, L2, L3)
            angles = compute_servo_angles(L1, L2, L3, claw_pos[0] - 1, claw_pos[1])
            if angles is not None:
                move_servo(0, angles[0])
                move_servo(1, angles[1])
                move_servo(2, angles[2])

        elif cmd == b'y':  # Claw Open
            move_servo(3, 45)  # Adjust angle as needed

        elif cmd == b'z':  # Claw Closed
            move_servo(3, 0)  # Adjust angle as needed

        elif cmd == b'd':  # Arm Up
            claw_pos = calculate_claw_position(L1, L2, L3)
            angles = compute_servo_angles(L1, L2, L3, claw_pos[0], claw_pos[1] + 1)
            if angles is not None:
                move_servo(0, angles[0])
                move_servo(1, angles[1])
                move_servo(2, angles[2])

        elif cmd == b'e':  # Arm Down
            claw_pos = calculate_claw_position(L1, L2, L3)
            angles = compute_servo_angles(L1, L2, L3, claw_pos[0], claw_pos[1] - 1)
            if angles is not None:
                move_servo(0, angles[0])
                move_servo(1, angles[1])
                move_servo(2, angles[2])

        else:
            print("Unknown command:", cmd)

    except Exception as e:
        print("move_robot error:", e)


# --- Main Loop (Radio Listening) ---
while True:
    if radio.available():
        buf = radio.read(1)           # returns a bytearray of length 1
        print("RX got →", buf)
        move_robot(buf)               # pass it into move_robot
    time.sleep(0.05)


