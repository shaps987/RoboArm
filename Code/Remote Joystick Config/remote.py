from usmbus import SMBus
import time

# Initialize I2C bus (bus 0 on Pico)
bus = SMBus(0)
# ADS7830 command bytes for channels 0 to 7 (do not change these)
ads7830_commands = (0x84, 0xc4, 0x94, 0xd4, 0xa4, 0xe4, 0xb4, 0xf4)

def read_ads7830(channel):
    """
    Read ADC input using ADS7830.
    Note: Using address 0x48 based on your scan results.
    """
    bus.write_byte(0x48, ads7830_commands[channel])
    # A small delay might be needed between write and read
    time.sleep_us(200)
    return bus.read_byte(0x48)

while True:
    # Joystick 1: Chassis forward/backward on channel 6
    j1_v = read_ads7830(7)
    if j1_v < 110:
        print("Joystick 1: Chassis Forward")
    elif j1_v > 140:
        print("Joystick 1: Chassis Backward")
    
    # Joystick 1: Strafe left/right on channel 7
    j1_h = read_ads7830(6)
    if j1_h < 110:
       print("Joystick 1: Strafe Right")
    elif j1_h > 140:
       print("Joystick 1: Strafe Left")
    
    # Joystick 2: Chassis turning on channel 5
    j2 = read_ads7830(4)
    if j2 < 110:
        print("Joystick 2: Chassis Turning Right")
    elif j2 > 140:
        print("Joystick 2: Chassis Turning Left")
    
    # Joystick 3: Arm forward/backward on channel 2
    j3_v = read_ads7830(3)
    if j3_v < 110:
        print("Joystick 3: Arm Forward")
    elif j3_v > 140:
        print("Joystick 3: Arm Backward")
    
    # Joystick 3: Claw open/closed on channel 3
    j3_h = read_ads7830(2)
    if j3_h > 140:
        print("Joystick 3: Claw Open")
    elif 110 < j3_h < 140:
        print("Joystick 3: Claw Closed")
    
    # Joystick 4: Arm up/down on channel 0
    j4 = read_ads7830(0)
    if j4 > 140:
        print("Joystick 4: Arm Up")
    elif j4 < 110:
        print("Joystick 4: Arm Down")
    
    print("-----------------------------")
    time.sleep(0.5)

