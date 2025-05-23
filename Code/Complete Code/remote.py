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
    # read all four axes:
    fb     = read_ads7830(1)
    strafe = read_ads7830(0)
    turn   = read_ads7830(2)
    arm_fb = read_ads7830(5)
    claw   = read_ads7830(4)
    arm_ud = read_ads7830(7)

    # decide on one-byte cmd:
    cmd = None
    if   fb     > 128+DEADZONE: cmd = b"b"
    elif fb     < 128-DEADZONE: cmd = b"f"
    if   strafe > 128+DEADZONE: cmd = b"l"
    elif strafe < 128-DEADZONE: cmd = b"r"
    if   turn   > 128+DEADZONE: cmd = b"p"
    elif turn   < 128-DEADZONE: cmd = b"q"
    if   arm_fb > 128+DEADZONE: cmd = b"c"
    elif arm_fb < 128-DEADZONE: cmd = b"a"
    if   claw   > 128+DEADZONE: cmd = b"y"
    elif claw   < 128-DEADZONE: cmd = b"z"
    if   arm_ud > 128+DEADZONE: cmd = b"e"
    elif arm_ud < 128-DEADZONE: cmd = b"d"

    # if stick is centered now but wasn’t before, send “stop”
    if cmd is None and last_cmd is not None:
       cmd = b"s"

    # only send when the stick moves out of the dead-zone
    if cmd is not None and cmd != last_cmd:
        radio.send(cmd)
        print("TX →", cmd)
        last_cmd = cmd

    time.sleep(0.1)