# Robotic Arm Wiring Description

---

### 12V 10.5Ah Battery(Power Source)

- **Amperage Usage**: Supplies ~7A for both buck converters.
- **Purpose**: Powers two buck converters, stepping down to **5V** for the logic circuits and **7V** for the motor inputs.
- **Connections**:
  - **Positive (+)**:
    - Connect to a **10A slow-blow fuse** (chosen because the total current draw from both buck converters is ~7A, allowing a safe margin for surges).
    - From the fuse, connect to a **rocker switch** using the wires pre-soldered to the switch.
    - After the switch, split the positive line into two separate connections for the two buck converters.
    - Use 18 AWG wire for connections up to the switch and split point.
  - **Negative (-)**:
    - Split the negative line and connect to the negative input of both buck converters.
    - Use 14 AWG wire to minimize voltage drop.

---

### Buck Converter #1 (5V for Logic Circuits)

- **Amperage Usage**: Draws ~5.4A at 5V.
  - **Breakdown**:
    - **L298N Logic**: ~0.1A (50mA × 2 motor drivers).
    - **Raspberry Pi Pico**: ~0.2A.
    - **PCA9685 Servo Controller (Logic)**: ~0.3A.
    - **Servos (MG996R)**: ~4.8A (1.2A × 4 servos).
    - **Momentary LED Push Button**: ~20mA when lit.
- **Purpose**: Steps down 12V to 5V for powering the Raspberry Pi Pico, PCA9685, and L298N logic circuits.
- **Connections**:
  - **Input**:
    - **Positive (+)**: Connect to the + terminal of the 12V battery via the split positive wire.
    - **Negative (-)**: Connect to the negative terminal of the 12V battery via the split negative wire.
    - Use 14 AWG wires for input.
  - **Output (5V)**:
    - Pass through a **5A slow-blow fuse** (chosen because the total current draw of the Raspberry Pi Pico, PCA9685, and L298N logic inputs is ~3.7A at 5V, with additional headroom for safety).
    - Use 14 AWG ~10 foot long cables to bring power to the moving robot from the buck converter.
      - Connect to a power distribution block:
        - **Positive (+)**:
          - The 5V logic input of both L298Ns.
          - The 5V input of the Raspberry Pi Pico.
          - The 5V input of the PCA9685 servo controller.
          - Use 18 AWG wires for these connections.
        - **Negative (-)**:
          - The GND terminal on both L298Ns.
          - The GND wire for the Raspberry Pi Pico's power.
          - The GND terminal on the PCA9685 servo controller.
          - Use 18 AWG wires for these connections.

---

### Buck Converter #2 (7V for Motor Inputs)

- **Amperage Usage**: Supplies ~4.8A for motor inputs.
- **Purpose**: Steps down 12V to 7V for powering the VIN inputs on both L298Ns.
- **Connections**:
  - **Input**:
    - **Positive (+)**:
      - Connect to the + terminal of the 12V battery via the split positive wire.
      - Use **14 AWG wires** for this connection.
    - **Negative (-)**:
      - Connect to the negative terminal of the 12V battery via the split negative wire.
      - Use **14 AWG wires** for this connection.
  - **Output (7V)**:
    - **Positive (+)**:
      - Split the **14 AWG wire** into two **18 AWG wires**.
      - Pass each of the **18 AWG wires** through a **5A circuit breaker**, one for each motor driver.
      - Connect the output of each breaker to the VIN input of its respective **L298N motor driver**.
    - **Negative (-)**:
      - Split the negative wire and connect to the GND terminals of both motor drivers using **18 AWG wire**.

---

### Momentary Push Button with LED

- **Amperage Usage**: The LED draws ~20mA when lit.
- **Purpose**: Allows the user to safely stop the script and shut down the Raspberry Pi.
- **Connections**:
  - **Positive (+)**:
    - Connect the LED positive terminal (red wire) to the 5V output of the power distribution block.
  - **Negative (-)**:
    - Connect the LED negative terminal (black wire) to the GND on the power distribution block.
  - **NO (Blue)**:
    - Connect the Normally Open terminal to a GPIO pin on the Raspberry Pi Zero.

---

### L298N Motor Drivers

1. **L298N #1**:
   - **Amperage Usage**:
     - **VIN (Motor Power Input)**: Draws ~1.2A during normal operation to power two motors.
     - **5V Logic Input**: Draws ~0.05A (50mA) to power the logic circuitry.
   - **Connections**:
     - **VIN (Motor Power Input)**:
       - Connect to the 7V output of the second buck converter after its dedicated **5A slow-blow fuse**.
       - Use 14 AWG wire.
     - **5V Logic Input**:
       - Connect to the +5V output of the first buck converter after the combined **5A slow-blow fuse**.
       - Use 18 AWG wire.
     - **GND**:
       - Connect to the GND output of the first buck converter.
       - Connect to the GND output of the second buck converter.
       - Use 18 AWG wire.

2. **L298N #2**:
   - **Amperage Usage**:
     - **VIN (Motor Power Input)**: Draws ~1.2A during normal operation to power two motors.
     - **5V Logic Input**: Draws ~0.05A (50mA) to power the logic circuitry.
   - **Connections**:
     - **VIN (Motor Power Input)**:
       - Connect to the 7V output of the second buck converter after its dedicated **5A slow-blow fuse**.
       - Use 14 AWG wire.
     - **5V Logic Input**:
       - Connect to the +5V output of the first buck converter after the combined **5A slow-blow fuse**.
       - Use 18 AWG wire.
     - **GND**:
       - Connect to the GND output of the first buck converter.
       - Connect to the GND output of the second buck converter.
       - Use 18 AWG wire to connect to the respective 14 AWG wire bringing power from the buck converter to the robot.

---

### Raspberry Pi Pico

- **Purpose**: Controls the robotic arm's servos and motor drivers (L298Ns).
- **Connections**:
  - **5V Power Input**:
    - Connect to the +5V output of the first buck converter through the **power distribution block**.
    - Use 18 AWG wire.
  - **GND**:
    - Connect to the GND output of the first buck converter through the **power distribution block**.
    - Use 18 AWG wire.

---

### PCA9685 Servo Controller

- **Amperage Usage**: Draws ~0.3A at 5V (logic only; servos powered separately).
- **Purpose**: Controls the servos on the robotic arm.
- **Connections**:
  - **5V Power Input**:
    - Connect to the +5V output of the first buck converter through the **power distribution block**.
    - Use 18 AWG wire.
  - **GND**:
    - Connect to the GND output of the first buck converter through the **power distribution block**.
    - Use 18 AWG wire.
  - **Servo Pins**:
    - Connect the servos (e.g., MG996R) directly to the PCA9685.

---

### Servos (MG996R)

- **Amperage Usage**: Draws ~1.2A per servo at 5V (when loaded; ~0.5A idle).
- **Purpose**: Provide movement for the robotic arm joints and claw.
- **Connections**:
  - **Power, GND, and PWM**:
    - Powered and controlled directly by the PCA9685.
    - Servos come with built-in wires and connectors.

---

### Motors (TT Motors via L298N)

- **Amperage Usage**: Each motor draws ~1.2A at 7V (under normal load; peaks ~3A).
- **Purpose**: Drive the robot chassis with mecanum wheels.
- **Connections**:
  - Each motor is connected to one of the motor output terminals on the L298Ns:
    - **Motor 1**: L298N #1 Output 1 & Output 2.
    - **Motor 2**: L298N #1 Output 3 & Output 4.
    - **Motor 3**: L298N #2 Output 1 & Output 2.
    - **Motor 4**: L298N #2 Output 3 & Output 4.
  - Use 18 AWG wires for motor connections.

---

### Wiring Chart - THE FOLLOWING PINS ARE OUTDATED (THEY ARE FOR THE RPI ZERO VERSION, NOT THE PICO VERSION)
Voltage Distribution Unit:
  - Positive Terminal of Battery: To 10A Fuse
  - Negative Terminal of Battery: Splits and goes to the 2 buck converters
  - 10A Fuse: To switch
  - Switch: splits and goes to the 2 buck converters
  - 7V Buck Converter Positive Output: To robot, then splits into two wires. Each wire has a 5A fuse on it. And then each fuse goes into the 12V pin on its respective L298N
  - 7V Buck Converter Negative Output: To robot, then splits into two wires. Then each wire goes into the 12V pin on its respective L298N
Motor Driver 1:
  - ENA: Pin 32, GPIO12
  - IN1: Pin 29, GPIO5
  - IN2: Pin 31, GPIO6
  - ENB: Pin 33, GPIO13
  - IN3: Pin 35, GPIO19
  - IN4: Pin 37, GPIO26

Motor Driver 2:
  - ENA: Pin 12, GPIO18
  - IN1: Pin 16, GPIO23
  - IN2: Pin 18, GPIO24
  - ENB: Pin 22, GPIO25
  - IN3: Pin 13, GPIO27
  - IN4: Pin 15, GPIO22

PCA9685:
  - SDA: Pin 3, GPIO2
  - SCL: Pin 5, GPIO3
  - VCC: 3.3v pin on Raspberry Pi Pico
  - GND: Ground pin on Raspberry Pi PIco
  - V+ (on the PCA9685 terminal block): 5V distribution block
  - GND (on the PCA9685 terminal block): 5V distribution block

NRF24L01:
  - VCC: Pin 36
  - GND: Pin 38
  - CE: Pin 19, GPIO 14
  - CSN: Pin 20, GPIO 15
  - SCK: Pin 24, GPIO 18
  - MOSI: Pin 25, GPIO 19
  - MISO: Pin 21, GPIO 16
  - IRQ: Not used

5V Distribution Block:
  - +/- Input: 5V Buck Converter +/- Output
  - +/- Outputs:
    - To Raspberry Pi Pico via a Pigtail to Micro-USB cable
    - To L298N #1 5V Logic Input and GND terminals
    - To L298N #2 5V Logic Input and GND terminals
    - To PCA9685 +/- V+ terminal block

### Final Checklist

1. **12V Battery** powers both buck converters:
   - Buck Converter #1 (5V): Supplies the Raspberry Pi Pico, PCA9685, and L298N logic inputs.
   - Buck Converter #2 (7V): Supplies the VIN inputs (motor power) on both L298Ns.
2. **Fuses**:
   - **10A Slow-Blow Fuse**: Protects the entire circuit, chosen because the combined input current for both buck converters is ~7A.
   - **5A Slow-Blow Fuse** (Logic Circuit): Protects the Raspberry Pi Pico, PCA9685, and L298N logic inputs.
   - **5A Slow-Blow Fuses** (Motor Drivers): One fuse per L298N motor driver, chosen because each driver can handle up to ~4A continuously with peaks up to 5A during stalls or startups.
3. **Switch**:
   - Use an 18 AWG wire to connect the positive terminal of the battery to the switch and the switch to the fuse.
4. **Wiring to the Robot**:
   - Use 14 AWG wires for the long runs to the robot's power distribution blocks.
   - Inside the robot, use 18 AWG wires to connect components to the distribution blocks.

---
