# --- Imports --- #
import board, busio, digitalio, time
from circuitpython_nrf24l01.rf24 import RF24

# --- Initialize SPI and TX Pipe --- #
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
ce = digitalio.DigitalInOut(board.D1)
csn = digitalio.DigitalInOut(board.D0)
radio = RF24(spi, csn, ce)
radio.channel      = 100      # 0–125
radio.payload_size = 1        # one-byte packets
print("TX: channel =", radio.channel, "addr =", b"\xe1\xf0\xf0\xf0\xf0", 
      "payload_size =", radio.payload_size)
radio.open_tx_pipe(b'\xe1\xf0\xf0\xf0\xf0')
radio.listen = False

# I²C address discovered by scan
ADS7830_ADDR = 0x48

# --- I2C Setup for ADS7830 ---
i2c = busio.I2C(board.SCL, board.SDA)
ads7830_commands = (0x84, 0xC4, 0x94, 0xD4, 0xA4, 0xE4, 0xB4, 0xF4)

def read_ads7830(channel):
    """Read one byte from the given ADS7830 channel over I2C."""
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto(ADS7830_ADDR, bytes([ads7830_commands[channel]]))
        buf = bytearray(1)
        i2c.readfrom_into(ADS7830_ADDR, buf)
        return buf[0]
    except Exception as e:
        print("I2C error:", e)
        return 0
    finally:
        i2c.unlock()

# Dead-zone thresholds
CENTER   = 128
DEADZONE = 20
UPPER    = CENTER + DEADZONE
LOWER    = CENTER - DEADZONE

last_cmd = None
while True:
    # Read all four axes
    fb      = read_ads7830(1)
    strafe  = read_ads7830(0)
    turn    = read_ads7830(2)
    arm_fb  = read_ads7830(5)
    claw    = read_ads7830(4)
    arm_ud  = read_ads7830(7)

    # Decide on one-byte cmd
    cmd = None
    if   fb     > UPPER: cmd = b"b"
    elif fb     < LOWER: cmd = b"f"
    elif strafe > UPPER: cmd = b"l"
    elif strafe < LOWER: cmd = b"r"
    elif turn   > UPPER: cmd = b"p"
    elif turn   < LOWER: cmd = b"q"
    elif arm_fb > UPPER: cmd = b"c"
    elif arm_fb < LOWER: cmd = b"a"
    elif claw   > UPPER: cmd = b"y"
    elif claw   < LOWER: cmd = b"z"
    elif arm_ud > UPPER: cmd = b"e"
    elif arm_ud < LOWER: cmd = b"d"

    # LOGIC FIX: 
    if cmd is not None:
        # 1. If it's an arm/claw command (a, c, d, e, y, z), ALWAYS send it to keep moving
        if cmd in [b"a", b"c", b"d", b"e", b"y", b"z"]:
            radio.send(cmd)
            print("TX (Streaming) →", cmd)
        
        # 2. For driving/turning, only send if it's a change (prevents jitter)
        elif cmd != last_cmd:
            radio.send(cmd)
            print("TX (Chassis) →", cmd)
            
    # 3. If we just let go of the stick, send "stop" once
    elif last_cmd is not None:
        radio.send(b"s")
        print("TX → STOP")

    last_cmd = cmd
    
    # Speed this up slightly for smoother arm movement
    # 0.05 is usually the "sweet spot" for responsiveness
    time.sleep(0.05)
