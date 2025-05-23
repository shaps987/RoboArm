# Robotic Arm with nRF24L01⁺ (Pico ↔ nRF52840 Sense)

This README describes how to rewire and modify your project to use an **nRF24L01⁺** wireless transceiver instead of BLE, with:

- **Robot:** Raspberry Pi Pico running MicroPython  
- **Remote:** nRF52840 Sense running CircuitPython

---

## 1. Robot (Raspberry Pi Pico + MicroPython)

### 1.1 Wiring / Pin-out

| Function       | Pico GP# | Physical Pin | Notes            |
| -------------- | :------: | :----------: | ---------------- |
| **nRF24L01⁺**  |          |              |                  |
| VCC            | —        | 36           | 3.3 V            |
| GND            | —        | 38           | Ground           |
| CE             | GP14     | 19           | Digital GPIO     |
| CSN            | GP15     | 20           | Active-low CS    |
| SCK            | GP18     | 24           | SPI0 SCK         |
| MOSI           | GP19     | 25           | SPI0 MOSI        |
| MISO           | GP16     | 21           | SPI0 MISO        |
| **PCA9685**    |          |              |                  |
| SDA            | GP0      | 1            | I2C0 SDA         |
| SCL            | GP1      | 2            | I2C0 SCL         |
| **L298N #1**   |          |              | Controls 2 motors|
| IN1            | GP2      | 4            |                  |
| IN2            | GP3      | 5            |                  |
| IN3            | GP4      | 6            |                  |
| IN4            | GP5      | 7            |                  |
| ENA (PWM)      | GP6      | 9            | PWM0 CH0         |
| ENB (PWM)      | GP7      | 10           | PWM0 CH1         |
| **L298N #2**   |          |              | Controls 2 motors|
| IN1            | GP8      | 11           |                  |
| IN2            | GP9      | 12           |                  |
| IN3            | GP10     | 14           |                  |
| IN4            | GP11     | 15           |                  |
| ENA (PWM)      | GP12     | 16           | PWM0 CH2         |
| ENB (PWM)      | GP13     | 17           | PWM0 CH3         |

---

### 1.2 Code Edits

1. **Remove BLE-related imports and `asyncio` logic**  
```python
# Remove these:
from bleak import BleakScanner, BleakClient
import asyncio
# and any `async def` connect_to_remote(), peripheral_task(), etc.
```

2. **Import nRF24L01⁺ and initialize SPI and RX logic**  
```python
import board, busio, digitalio, time
from adafruit_nrf24l01 import NRF24L01

spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
ce = digitalio.DigitalInOut(board.GP14)
csn = digitalio.DigitalInOut(board.GP15)
radio = NRF24L01(spi, csn, ce, channel=100, payload_size=1)
radio.open_rx_pipe(0, b'\xe1\xf0\xf0\xf0\xf0')
```

3. **Replace BLE connection and main loop with a blocking `while True:` that reads from `radio`**  
```python
while True:
    if radio.available():
        buf = radio.read(1)
        cmd = bytes(buf)
        move_robot(cmd)
    time.sleep(0.1)
```

4. **Delete all BLE callbacks, `async def` functions, and BLE-specific error handling**


---

## 2. Remote (nRF52840 Sense + CircuitPython)

### 2.1 Wiring / Pin-out

| Function       | Board Pin    | Notes            |
| -------------- | ------------ | ---------------- |
| **ADS7830**    |              |                  |
| VCC            | 3V3          |                  |
| GND            | GND          |                  |
| SDA            | SDA          | I²C SDA          |
| SCL            | SCL          | I²C SCL          |
| **nRF24L01⁺**  |              |                  |
| VCC            | 3V3          |                  |
| GND            | GND          |                  |
| CE             | D5           | Digital IO       |
| CSN            | D6           | Active-low CS    |
| SCK            | SCK          | SPI SCK          |
| MOSI           | MOSI         | SPI MOSI         |
| MISO           | MISO         | SPI MISO         |

---

### 2.2 Code Edits

1. **Remove BLE-related imports and logic**  
```python
# Remove these:
from bleak import BleakScanner, BleakClient
import asyncio
# and any `async def` connect_to_remote(), peripheral_task(), etc.
```

2. **Import nRF24L01⁺ and initialize SPI and TX pipe**  
```python
import board, busio, digitalio, time
from adafruit_nrf24l01 import NRF24L01

spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
ce = digitalio.DigitalInOut(board.D5)
csn = digitalio.DigitalInOut(board.D6)
radio = NRF24L01(spi, csn, ce, channel=100, payload_size=1)
radio.open_tx_pipe(b'\xe1\xf0\xf0\xf0\xf0')
```

3. **Replace BLE `notify` calls with `radio.send()` logic inside a `while True:`**  
```python
last_cmd = None
while True:
    val = ads.read(1)  # joystick forward/back
    if val > 148:
        cmd = b"f"
    elif val < 108:
        cmd = b"b"
    else:
        cmd = None

    if cmd and cmd != last_cmd:
        radio.send(cmd)
        last_cmd = cmd

    time.sleep(0.1)
```

4. **Delete `async def` functions and all BLE-specific control flow**

---
