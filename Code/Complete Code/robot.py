import math
import board
import busio
import time
import pwmio
from adafruit_pca9685 import PCA9685
from digitalio import DigitalInOut, Direction
from circuitpython_nrf24l01.rf24 import RF24

# --- 1. Robot Arm Constants ---
l1, l2, l3 = 6.0, 6.0, 2.0  
current_x = 8.0
current_y = 6.0

# --- 2. Hardware Setup (I2C, SPI, Radio) ---
i2c = busio.I2C(board.GP1, board.GP0)
pca = PCA9685(i2c)
pca.frequency = 50

spi = busio.SPI(board.GP18, board.GP19, board.GP16)
ce = DigitalInOut(board.GP14); ce.direction = Direction.OUTPUT
csn = DigitalInOut(board.GP15); csn.direction = Direction.OUTPUT

radio = RF24(spi, csn, ce)
radio.channel = 100
radio.payload_size = 1
radio.open_rx_pipe(1, b'\xe1\xf0\xf0\xf0\xf0')
radio.listen = True

# --- 3. Motor Controller Setup (Explicit Pin Mapping) ---
in1_1 = DigitalInOut(board.GP2); in1_1.direction = Direction.OUTPUT
in2_1 = DigitalInOut(board.GP3); in2_1.direction = Direction.OUTPUT
in3_1 = DigitalInOut(board.GP4); in3_1.direction = Direction.OUTPUT
in4_1 = DigitalInOut(board.GP5); in4_1.direction = Direction.OUTPUT
ena_1 = pwmio.PWMOut(board.GP6, frequency=100); ena_1.duty_cycle = 49151
enb_1 = pwmio.PWMOut(board.GP7, frequency=100); enb_1.duty_cycle = 49151

in1_2 = DigitalInOut(board.GP8); in1_2.direction = Direction.OUTPUT
in2_2 = DigitalInOut(board.GP9); in2_2.direction = Direction.OUTPUT
in3_2 = DigitalInOut(board.GP10); in3_2.direction = Direction.OUTPUT
in4_2 = DigitalInOut(board.GP11); in4_2.direction = Direction.OUTPUT
ena_2 = pwmio.PWMOut(board.GP12, frequency=100); ena_2.duty_cycle = 49151
enb_2 = pwmio.PWMOut(board.GP13, frequency=100); enb_2.duty_cycle = 49151

all_motors = [in1_1, in2_1, in3_1, in4_1, in1_2, in2_2, in3_2, in4_2]

# --- 4. Core Functions ---
def set_servo(channel, angle):
    angle = max(0, min(180, angle))
    min_pulse, max_pulse = 500, 2500
    pulse_width = int(min_pulse + (angle / 180.0) * (max_pulse - min_pulse))
    duty_cycle = int((pulse_width / 1000000) * pca.frequency * 65536)
    pca.channels[channel].duty_cycle = duty_cycle

def calculate_mirrored_ik(x, y):
    x_adj = x - l3
    y_adj = y
    r_adj = math.sqrt(x_adj**2 + y_adj**2)

    # Law of Cosines
    cos_angle2 = (r_adj**2 - l1**2 - l2**2) / (2 * l1 * l2)
    angle2 = math.acos(max(-1.0, min(1.0, cos_angle2)))
    angle1 = math.atan2(y_adj, x_adj) - math.atan2(l2 * math.sin(angle2), l1 + l2 * math.cos(angle2))

    # Mirrored Logic
    joint2_x = l1 * math.cos(angle1) + l2 * math.cos(angle1 + angle2)
    joint2_y = l1 * math.sin(angle1) + l2 * math.sin(angle1 + angle2)
    line_angle = math.atan2(joint2_y, joint2_x)

    angle1 = 2 * line_angle - angle1
    angle2 = -angle2
    angle3 = -(angle1 + angle2)

    # Conversion to Degrees
    a1_deg, a2_deg, a3_deg = math.degrees(angle1), math.degrees(angle2), math.degrees(angle3)
    
    # Mapping based on your mirrored logic formulas
    s1 = 90 - (-1 * (90 - a1_deg))
    s2 = 90 + (-1 * (a2_deg + 90))
    s3 = 90 - a3_deg
    return s1, s2, s3

def move_robot(cmd):
    global current_x, current_y
    step = 0.5
    
    # --- Chassis Controls ---
    if cmd == b'f':
        in1_1.value, in2_1.value, in3_1.value, in4_1.value = False, True, True, False
        in1_2.value, in2_2.value, in3_2.value, in4_2.value = False, True, True, False
    elif cmd == b'b':
        in1_1.value, in2_1.value, in3_1.value, in4_1.value = True, False, False, True
        in1_2.value, in2_2.value, in3_2.value, in4_2.value = True, False, False, True
    elif cmd == b'r':  # Strafe Right
        in1_1.value, in2_1.value, in3_1.value, in4_1.value = True, False, True, False
        in1_2.value, in2_2.value, in3_2.value, in4_2.value = True, False, True, False
    elif cmd == b'l':  # Strafe Left
        in1_1.value, in2_1.value, in3_1.value, in4_1.value = False, True, False, True
        in1_2.value, in2_2.value, in3_2.value, in4_2.value = False, True, False, True
    elif cmd == b'q':  # turn right (in place)
        in1_1.value, in2_1.value, in3_1.value, in4_1.value = True, False, False, True
        in1_2.value, in2_2.value, in3_2.value, in4_2.value = False, True, True, False

    elif cmd == b'p':  # turn left (in place)
        in1_1.value, in2_1.value, in3_1.value, in4_1.value = False, True, True, False
        in1_2.value, in2_2.value, in3_2.value, in4_2.value = True, False, False, True

    elif cmd == b"s":   # STOP
        in1_1.value = in2_1.value = in3_1.value = in4_1.value = False
        in1_2.value = in2_2.value = in3_2.value = in4_2.value = False

    # --- Arm Movement (Mirrored IK) ---
    elif cmd in [b'a', b'c', b'd', b'e']:
        if cmd == b'a': current_x += step
        elif cmd == b'c': current_x -= step
        elif cmd == b'd': current_y += step
        elif cmd == b'e': current_y -= step
        
        # Reach limit clamp (Total length is 12)
        current_x = max(2.0, min(11.5, current_x))
        current_y = max(1.0, min(11.0, current_y))
        
        s1, s2, s3 = calculate_mirrored_ik(current_x, current_y)
        set_servo(12, s1)
        set_servo(13, s2)
        set_servo(14, s3)

    elif cmd == b'y': set_servo(15, 45) # Claw Open
    elif cmd == b'z': set_servo(15, 0)  # Claw Closed

# --- 5. Main Loop ---
print("System Ready.")
# Sync servos to start position
set_servo(12, 90); set_servo(13, 90); set_servo(14, 90)

while True:
    if radio.available():
        # While there is data in the radio's hardware buffer...
        while radio.available():
            buf = radio.read(1)
            move_robot(buf)
            # Small delay so it doesn't move TOO fast while clearing the buffer
            time.sleep(0.005) 
    else:
        # If no signal is received, we can optionally stop the chassis
        # but keep this light so the arm doesn't jitter.
        pass
    
    time.sleep(0.01)
