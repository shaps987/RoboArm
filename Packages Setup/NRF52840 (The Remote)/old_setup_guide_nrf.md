# nRF52840 BLE Project Setup with ADS7830 and CircuitPython

This guide walks you through flashing firmware, setting up libraries, wiring the hardware, and loading the code for your nRF52840 board running CircuitPython with BLE support and an ADS7830 sensor.

---

## 1. Flash CircuitPython Firmware

1. **Download Firmware:**
   - Visit [CircuitPython Downloads](https://circuitpython.org/downloads) and locate the appropriate UF2 file for your nRF52840 board.
   - **Note:** If you're using a board like the Adafruit nRF52840 Sense, use its UF2 file. For other variants (e.g., Seeed Studio XIAO nRF52840), you may need a custom or community-provided firmware.

2. **Enter Bootloader Mode:**
   - For many nRF52840 boards, double‑tap the reset button. The board will appear as a USB mass storage device (commonly named **CIRCUITPY**).

3. **Copy the UF2 File:**
   - Drag and drop the UF2 file onto the board’s drive. The board will reboot into CircuitPython.

---

## 2. Prepare Your Libraries

1. **Create or Update the `lib` Folder:**
   - On the **CIRCUITPY** drive, ensure there is a folder named `lib`.

2. **Download Required Libraries:**
   - Just take the pre-made lib folder in this github repository (its in packages setup -> nrf52840 (the remote) -> files -> lib)

---

## 3. Hardware Wiring

### ADS7830 Sensor
- **Power:** Connect the ADS7830 VDD to your board’s 3.3V output and GND to the board’s ground.
- **I2C Lines:**
  - **SDA:** Connect ADS7830 SDA to the board’s SDA pin (usually referenced as `board.SDA`).
  - **SCL:** Connect ADS7830 SCL to the board’s SCL pin (usually referenced as `board.SCL`).

### LED Indicator (Optional)
- Most boards include a built-in LED referenced as `board.LED`. Confirm with your board's documentation.

---

## 4. Write and Load Your Code

1. **Create Your Main Script:**
   - On the **CIRCUITPY** drive, create a file named `code.py`.

2. **Paste the Following Code into `code.py`:**

   ```python
    import board
    import digitalio
    import busio
    import microcontroller
    import asyncio

    # --- BLE Imports from Adafruit Libraries ---
    from adafruit_ble import BLERadio, Service
    from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
    from adafruit_ble.characteristics import Characteristic
    from adafruit_ble.uuid import StandardUUID
    from adafruit_ble.services.standard.device_info import DeviceInfoService

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

    def uid():
        """Return the unique id of the device as a string."""
        return "".join("{:02x}".format(x) for x in microcontroller.cpu.uid)

    # --- BLE Constants ---
    _BLE_APPEARANCE_GENERIC_REMOTE_CONTROL = 384

    # --- Define a Custom BLE Service ---
    class RemoteService(Service):
        # 16-bit service UUID is fine here
        uuid = StandardUUID(0x1848)

        # use StandardUUID for the char, and override to 1 byte
        button = Characteristic(
            uuid=StandardUUID(0x2A6E),                     # spec-assigned 16-bit
            properties=Characteristic.READ | Characteristic.NOTIFY,
            max_length=1                                    # allow exactly one byte
        )

    remote_service = RemoteService()
    device_info = DeviceInfoService(
        manufacturer="RoboticArmRemote",
        model_number="1.0",
        serial_number=uid(),
        hardware_revision="1.0",
        firmware_revision="1.0",
    )

    # --- Hardware LED Setup ---
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT

    # Global BLE state
    ble = BLERadio()
    connected = False

    # --- BLE Peripheral Task ---
    async def peripheral_task():
        global connected
        advertisement = ProvideServicesAdvertisement(remote_service, device_info)
        advertisement.appearance = _BLE_APPEARANCE_GENERIC_REMOTE_CONTROL
        advertisement.complete_name = "RoboticArmRemote"
        ble.start_advertising(advertisement)
        while True:
            if ble.connected:
                conn = ble.connections[0]
                connected = True
                print("Connected to:", conn)
                while ble.connected:
                    await asyncio.sleep(0.1)
                connected = False
                print("Disconnected")
                ble.start_advertising(advertisement)
            await asyncio.sleep(0.1)

    # Dead-zone thresholds
    CENTER   = 128
    DEADZONE = 20
    UPPER    = CENTER + DEADZONE
    LOWER    = CENTER - DEADZONE
    _last_button = None

    # --- Remote Task ---
    async def remote_task():
        global connected, _last_button
        while True:
            if not connected:
                print("Not Connected")
                await asyncio.sleep(1)
                continue

            try:
                # Read & map exactly like before...
                chassis_fb     = read_ads7830(1)
                chassis_strafe = read_ads7830(0)
                chassis_turn   = read_ads7830(2)
                arm_fb         = read_ads7830(5)
                claw           = read_ads7830(4)
                arm_ud         = read_ads7830(7)

                # Decide which single-byte command to send
                cmd = None
                if chassis_fb >  UPPER:
                    print("Joystick 1 → Chassis Forward")
                    cmd = b"f"
                elif chassis_fb <  LOWER:
                    print("Joystick 1 → Chassis Backward")
                    cmd = b"b"

                if chassis_strafe >  UPPER:
                    print("Joystick 1 → Chassis Strafe Right")
                    cmd = b"r"
                elif chassis_strafe <  LOWER:
                    print("Joystick 1 → Chassis Strafe Left")
                    cmd = b"l"

                if chassis_turn >  UPPER:
                    print("Joystick 2 → Chassis Turning Left")
                    cmd = b"p"
                elif chassis_turn <  LOWER:
                    print("Joystick 2 → Chassis Turning Right")
                    cmd = b"q"

                if arm_fb >  UPPER:
                    print("Joystick 3 → Arm Backward")
                    cmd = b"c"
                elif arm_fb <  LOWER:
                    print("Joystick 3 → Arm Forward")
                    cmd = b"a"

                if claw >  UPPER:
                    print("Joystick 3 → Claw Open")
                    cmd = b"y"
                elif claw <  LOWER:
                    print("Joystick 3 → Claw Closed")
                    cmd = b"z"

                if arm_ud >  UPPER:
                    print("Joystick 4 → Arm Down")
                    cmd = b"e"
                elif arm_ud <  LOWER:
                    print("Joystick 4 → Arm Up")
                    cmd = b"d"

                # If we got a new command and it's different, send it
                if cmd is not None and cmd != _last_button:
                    remote_service.button = cmd
                    _last_button = cmd

            except Exception as e:
                print("Error in remote_task:", e)

            await asyncio.sleep(0.1)

    # --- LED Blink Task ---
    async def blink_task():
        toggle = True
        while True:
            led.value = toggle
            toggle = not toggle
            await asyncio.sleep(1.0 if connected else 0.25)

    # --- Main Entry Point ---
    async def main():
        await asyncio.gather(
            peripheral_task(),
            remote_task(),
            blink_task(),
        )

    if __name__ == "__main__":
        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            print("Exiting…")
        finally:
            print("Cleaning up…")