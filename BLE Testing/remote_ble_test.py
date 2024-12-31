import aioble
import bluetooth
import asyncio

_GENERIC = bluetooth.UUID(0x1848)
_BUTTON_UUID = bluetooth.UUID(0x2A6E)

remote_service = aioble.Service(_GENERIC)
button_characteristic = aioble.Characteristic(remote_service, _BUTTON_UUID, read=True, notify=True)

print("Registering services")
aioble.register_services(remote_service)

async def advertise_remote():
    while True:
        print("Not Connected")
        try:
            async with await aioble.advertise(
                250_000,
                name="RoboticArmRemote",
                services=[_GENERIC],
            ) as connection:
                print(f"Connected to: {connection.device}")
                while True:
                    await asyncio.sleep(1)  # Maintain connection
        except asyncio.CancelledError:
            break  # Handle task cancellation if necessary

async def main():
    print("Starting BLE remote...")
    await advertise_remote()

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Exiting...")