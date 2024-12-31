from bleak import BleakScanner, BleakClient
import asyncio

REMOTE_NAME = "RoboticArmRemote"
REMOTE_UUID = "1848"  # Service UUID
COMMAND_UUID = "2A6E"  # Characteristic UUID

async def find_remote():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name == REMOTE_NAME:
            print(f"Found remote: {device}")
            return device
    print("Remote not found.")
    return None

async def connect_to_remote():
    device = await find_remote()
    if not device:
        print("Remote not found. Retrying...")
        return

    async with BleakClient(device) as client:
        print(f"Connected to remote: {device.name}")
        try:
            # Ensure the remote service is available
            services = client.services
            print(f"Available services: {services}")
            
            # Subscribe to the characteristic
            def notification_handler(sender, data):
                print(f"Notification from {sender}: {data}")

            await client.start_notify(COMMAND_UUID, notification_handler)
            print(f"Subscribed to {COMMAND_UUID}. Waiting for notifications...")

            # Keep the connection alive
            while True:
                await asyncio.sleep(1)

        except Exception as e:
            print(f"Error: {e}")

async def main():
    while True:
        await connect_to_remote()
        await asyncio.sleep(2)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Exiting...")