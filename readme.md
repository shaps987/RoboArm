# Robotic Arm Project

## Overview
This project involves building a versatile and remotely controlled robotic arm, leveraging the power and flexibility of Raspberry Pi microcontrollers. The robotic arm is designed for precise movement and versatility, controlled remotely via a custom-built controller featuring multiple joysticks. The remote communicates with the robotic arm over Bluetooth Low Energy (BLE) for real-time command execution.

The **remote control** is powered by a Raspberry Pi Pico WH and includes an analog-to-digital converter to process joystick inputs. These joysticks allow intuitive control of the robotic arm's movements, including its base, joints, and claw.

The **robotic arm** itself is powered by a Raspberry Pi Zero WH, which coordinates the movements of servos and motors to execute commands received from the remote. With **inverse kinematics**, the robotic arm achieves smooth and accurate movement for tasks requiring precision. The arm is mounted on a mecanum wheel chassis, enabling it to move in all directions and navigate complex environments.

---

## Components

### Remote Control
- **Raspberry Pi Pico WH**: Used as the main controller for the remote.
- **Analog to Digital Converter (Adafruit ADS7830)**: Converts the analog signals from the joysticks to digital signals.
- **4 Joysticks**: Used to control the movement of the robotic arm.

### Robotic Arm
- **Raspberry Pi Zero WH**: Used as the main controller for the robotic arm.
- **4 Servos (MG996R)**:
  - 3 for the joints of the arm.
  - 1 for the claw's grippers.
- **2 Motor Controllers (L298N)**: Control a total of 4 mecanum wheel motors for the robotic arm's chassis.
- **Servo Controller (PCA9685)**: Controls the 4 servos.
- **1 12V Battery**: Powers the entire robotic arm system.
- **2 Buck Converters**:
  - One steps down from **12V to 5V** for logic circuits.
  - One steps down from **12V to 7V** for motor power.
- **Various Protection Devices**:
  - 1 **10A Slow-Blow Fuse** for the entire circuit.
  - 1 **5A Slow-Blow Fuse** for the logic circuit (RPi Zero, PCA9685 servo controller, and the logic inputs of both L298N motor drivers).
  - 2 **5A Circuit Breakers** for the motor circuits (one for each L298N).
- **Rocker Switch**

---

## Power Supply

### Remote Control
- Powered by a 6V 4 AA battery pack stepped down to 5V with a buck converter.

### Robotic Arm
- **1 12V Battery**:
  - The positive terminal is connected to a **10A slow-blow fuse**, chosen because the combined input current for the buck converters is ~7A, allowing for surges.
  - Powers two buck converters:
    - **Buck Converter #1 (5V)**:
      - Supplies the Raspberry Pi Zero, PCA9685, and logic inputs of both L298Ns.
      - Protected by a **5A slow-blow fuse**, chosen because the total current draw of these components is ~3.7A, with additional headroom for safety.
    - **Buck Converter #2 (7V)**:
      - Supplies the VIN inputs of both L298Ns for motor power.
      - Protected by **two 5A circuit breakers** (one per L298N motor driver, chosen because each driver can handle up to ~4A under normal operation, with peaks up to 5A during stalls or startups).

---

## Setup

### 1. Purchase Products
Here are the product links with their prices (prices may change):
| **Item**                                                                                                               | **Price**  |
|------------------------------------------------------------------------------------------------------------------------|------------|
| [Chrome Battery YTX12-BS iGel](https://www.amazon.com/Chrome-Battery-iGel-12-BS-CYTX12-BS/dp/B01DOIULOO)                | $39.90     |
| [Inline Blade Fuse Holder Set](https://www.amazon.com/GNAWRISHING-Inline-Holder-Standard-Assorted/dp/B09Y8GB799)       | $8.99      |
| [5A Circuit Breaker (2-Pack)](https://www.amazon.com/mxuteuk-Breakers-Protector-Terminals-Waterproof/dp/B08QVCYLMD)    | $9.99      |
| [Rocker Switch](https://www.amazon.com/gp/product/B07XD8J2PL)                                                          | $8.99      |
| [12V to 5V Buck Converter](https://www.amazon.com/VOLRANTISE-Converter-Voltage-Regulator-Transformer/dp/B09X1ZGR38)       | $11.98     |
| [Adjustable Buck Converter (12V to 7V)](https://www.amazon.com/Converter-DROK-Adjustable-Regulator-Indicator/dp/B00HHQ0VP4) | $16.11  |
| [Power Distribution Board](https://www.amazon.com/Tnisesm-Position-Distribution-Inputs-Outputs/dp/B0BWN19R6R)          | $8.99      |
| [Micro USB to Pigtail Cable](https://www.amazon.com/gp/product/B0BZ8XWL18/ref=ewc_pr_img_4?smid=A1PP7IW2YCJN73&psc=1)   | $9.59      |
| [Raspberry Pi Zero WH](https://www.amazon.com/Raspberry-Bluetooth-Compatible-Connector-headers/dp/B0CG99MR5W)          | $21.98     |
| [Momentary Push Button Switch with LED (5-pack)](https://www.amazon.com/Momentary-Indicator-Pushbutton-Waterproof-Stainless/dp/B08L13JG8M/) | $13.98   |
| [L298N Motor Drivers (4-pack)](https://www.amazon.com/gp/product/B0CR6MLY7F/ref=ewc_pr_img_7?smid=A1YZW40LYQY3L1&psc=1) | $9.99      |
| [Mecanum Wheels and Motors (4-pack)](https://www.amazon.com/gp/product/B0CBJJM1KQ/ref=ewc_pr_img_3?smid=A2S878EGP30YQG&th=1) | $15.60  |
| [PCA9685 Servo Driver (2-pack)](https://www.amazon.com/gp/product/B07BRS249H/ref=ewc_pr_img_8?smid=A30QSGOJR8LMXA&psc=1) | $13.99  |
| [MG996R Servos (4-Pack)](https://www.amazon.com/2-Pack-MG996R-Torque-Digital-Helicopter/dp/B0D7M2Y2BR)                 | $16.89     |
| [Adafruit 4 AA Battery Pack](https://www.adafruit.com/product/830)                                                     | $2.95      |
| [Adafruit Raspberry Pi Pico WH](https://www.adafruit.com/product/5544)                                                 | $7.00      |
| [Adafruit ADS7830 Analog to Digital Converter](https://www.adafruit.com/product/5836)                                  | $5.95      |
| [Joysticks (6-pack)](https://www.amazon.com/gp/product/B0DH2G1PVP/ref=ewc_pr_img_6?smid=A3EW69KEBHQF24&psc=1)           | $5.99      |

#### **Total Price: ~$230**

### 2. **Assemble the Remote Control**

- Connect the joysticks to the analog-to-digital converter.
- Connect the converter to the Raspberry Pi Pico.
- Power the Pico with the 6V battery pack stepped down to 5V.

### 3. **Assemble the Robotic Arm**
   - Connect the servos to the servo controller.
   - Connect the motor controllers to the mecanum wheel motors.
   - Connect the servo controller and motor controllers to the Raspberry Pi Zero.
   - Wire the **12V battery** as follows:
     - Connect the positive terminal to a **10A slow-blow fuse** then to a **rocker switch** and then split to two buck converters.
     - **Buck Converter #1 (5V)**:
       - Connect the positive output to the Raspberry Pi Zero, PCA9685, and logic inputs of both L298Ns.
       - Use a **5A slow-blow fuse** after the buck converter.
     - **Buck Converter #2 (7V)**:
       - Split the positive output into two separate wires, each passing through a **5A circuit breaker** before connecting to the VIN inputs of each L298N motor driver.
       - Connect all grounds (battery, buck converters, Raspberry Pi Zero, PCA9685, and L298Ns) to a common ground point.
   - Use **14 AWG wires** for connections between the battery and the buck converters and from the buck converters along the ~10 feet (one-way) long powerline to the moving robot.
   - Use **18 AWG wires** for connections from the power distribution board to the components inside the robot.

---

## Usage
- Use the joysticks on the remote control to send signals to the Raspberry Pi Zero.
- The Raspberry Pi Zero processes these signals to control the servos and motors, moving the robotic arm accordingly.

---

## Future Improvements
- Add more sensors for better control and feedback.
- Enhance the software for more precise movements and automation.

---

## License
This project is licensed under the MIT License.

---

## Acknowledgements
Special thanks to the wonderful internet for providing many valuable resources for this project.

---