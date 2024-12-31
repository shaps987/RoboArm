# --- Import Required Libraries ---
from usmbus import SMBus
import aioble
import bluetooth
import machine
import uasyncio as asyncio
from micropython import const

# --- SMBus and Device Constants ---
bus = SMBus(0)  # Use 0 for the I2C bus on Pico
ads7830_commands = (0x84, 0xc4, 0x94, 0xd4, 0xa4, 0xe4, 0xb4, 0xf4)

# --- Analog-to-Digital Conversion Function ---
def read_ads7830(input):
    """ Read ADC input using ADS7830 """
    bus.write_byte(0x4b, ads7830_commands[input])  # Replace with your I2C address
    return bus.read_byte(0x4b)

# --- Device Unique ID ---
def uid():
    """ Return the unique id of the device as a string """
    return "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}".format(*machine.unique_id())

# --- BLE Constants ---
_GENERIC = bluetooth.UUID(0x1848)
_BUTTON_UUID = bluetooth.UUID(0x2A6E)
_DEVICE_INFO_UUID = bluetooth.UUID(0x180A)
_BLE_APPEARANCE_GENERIC_REMOTE_CONTROL = const(384)

# --- Hardware Setup ---
led = machine.Pin("LED", machine.Pin.OUT)

# --- BLE Service and Characteristics ---
device_info = aioble.Service(_DEVICE_INFO_UUID)
aioble.Characteristic(device_info, bluetooth.UUID(const(0x02A29)), read=True, initial="RoboticArmRemote")
aioble.Characteristic(device_info, bluetooth.UUID(const(0x2A24)), read=True, initial="1.0")
aioble.Characteristic(device_info, bluetooth.UUID(const(0x2A25)), read=True, initial=uid())
aioble.Characteristic(device_info, bluetooth.UUID(const(0x2A26)), read=True, initial="1.0")
aioble.Characteristic(device_info, bluetooth.UUID(const(0x2A28)), read=True, initial="1.0")

remote_service = aioble.Service(_GENERIC)
button_characteristic = aioble.Characteristic(remote_service, _BUTTON_UUID, read=True, notify=True)

# --- BLE Service Registration ---
print("Registering services")
aioble.register_services(remote_service, device_info)

# --- Connection State ---
connected = False
connection = None

# --- Remote Task ---
async def remote_task():
    """ Handle remote control commands """
    while True:
        if not connected:
            print("Not Connected")
            await asyncio.sleep(1)
            continue
        try:
            if read_ads7830(6) > 140:
                print(f"Joystick 1 is set to Chassis Forward, connection is: {connection}")
                button_characteristic.write(b"f")
                button_characteristic.notify(connection, b"f")
            elif read_ads7830(6) < 110:
                print(f"Joystick 1 is set to Chassis Backward, connection is: {connection}")
                button_characteristic.write(b"b")
                button_characteristic.notify(connection, b"b")
            elif read_ads7830(7) > 140:
                print(f"Joystick 1 is set to Chassis Strafe Right, connection is: {connection}")
                button_characteristic.write(b"r")
                button_characteristic.notify(connection, b"r")
            elif read_ads7830(7) < 110:
                print(f"Joystick 1 is set to Chassis Strafe Left, connection is: {connection}")
                button_characteristic.write(b"l")
                button_characteristic.notify(connection, b"l")
            elif read_ads7830(5) > 140:
                print(f"Joystick 2 is set to Chassis Turning Right, connection is: {connection}")
                button_characteristic.write(b"q")
                button_characteristic.notify(connection, b"q")
            elif read_ads7830(5) < 110:
                print(f"Joystick 2 is set to Chassis Turning Left, connection is: {connection}")
                button_characteristic.write(b"p")
                button_characteristic.notify(connection, b"p")
            elif read_ads7830(2) > 140:
                print(f"Joystick 3 is set to Arm Forward, connection is: {connection}")
                button_characteristic.write(b"a")
                button_characteristic.notify(connection, b"a")
            elif read_ads7830(2) < 110:
                print(f"Joystick 3 is set to Arm Backward, connection is: {connection}")
                button_characteristic.write(b"c")
                button_characteristic.notify(connection, b"c")
            elif read_ads7830(3) > 140:
                print(f"Joystick 3 is set to Claw Open, connection is: {connection}")
                button_characteristic.write(b"y")
                button_characteristic.notify(connection, b"y")
            elif read_ads7830(3) < 140 and read_ads7830(3) > 110:
                print(f"Joystick 3 is set to Claw Closed, connection is: {connection}")
                button_characteristic.write(b"z")
                button_characteristic.notify(connection, b"z")
            elif read_ads7830(0) > 140:
                print(f"Joystick 4 is set to Arm Up, connection is: {connection}")
                button_characteristic.write(b"d")
                button_characteristic.notify(connection, b"d")
            elif read_ads7830(0) < 110:
                print(f"Joystick 4 is set to Arm Down, connection is: {connection}")
                button_characteristic.write(b"e")
                button_characteristic.notify(connection, b"e")
            else:
                button_characteristic.write(b"!")  # Default state
        except Exception as e:
            print(f"Error reading joysticks: {e}")
        await asyncio.sleep(0.1)

# --- Peripheral Task ---
async def peripheral_task():
    """ Manage BLE connections """
    global connected, connection
    while True:
        connected = False
        async with await aioble.advertise(
            250_000,
            name="RoboticArmRemote",
            appearance=_BLE_APPEARANCE_GENERIC_REMOTE_CONTROL,
            services=[_GENERIC],
        ) as connection:
            print(f"Connected to: {connection.device}")
            connected = True
            await connection.disconnected()
            print("Disconnected")

# --- LED Blink Task ---
async def blink_task():
    """ Blink LED to show connection status """
    toggle = True
    while True:
        led.value(toggle)
        toggle = not toggle
        await asyncio.sleep_ms(250 if not connected else 1000)

# --- Main Function ---
async def main():
    tasks = [
        asyncio.create_task(peripheral_task()),
        asyncio.create_task(remote_task()),
        asyncio.create_task(blink_task()),
    ]
    try:
        # Run all tasks concurrently
        await asyncio.gather(*tasks)
    finally:
        # Ensure proper cleanup
        print("Cleaning up tasks...")
        for task in tasks:
            task.cancel()  # Cancel all tasks
        await asyncio.gather(*tasks, return_exceptions=True)  # Wait for tasks to cancel
        print("All tasks cleaned up.")

# --- Entry Point ---
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Exiting...")
finally:
    print("Exiting program. Cleaning up resources if necessary.")