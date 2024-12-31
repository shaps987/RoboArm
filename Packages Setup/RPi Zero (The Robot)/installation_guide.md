# Installing Libraries and Setting Up Startup Script for Robot Script

## Step 1: Update and Upgrade System
Ensure the system is updated:
```bash
sudo apt update && sudo apt upgrade -y
```

---

## Step 2: Install Python 3 and Essential Tools
Install Python 3 and pip if not already installed:
```bash
sudo apt install python3 python3-pip python3-venv -y
```

---

## Step 3: Set Up a Virtual Environment
Create and activate a virtual environment:
```bash
python3 -m venv myenv
source myenv/bin/activate
```

---

## Step 4: Install Required Libraries

### 4.1 Install NumPy
Install NumPy for numerical computations:
```bash
pip install numpy
```

### 4.2 Install Adafruit PCA9685 Library
Install the Adafruit PCA9685 library for servo control:
```bash
pip install adafruit-circuitpython-pca9685
```

### 4.3 Install busio and board Modules
Install the Adafruit Blinka package, which includes `busio` and `board` modules for CircuitPython compatibility:
```bash
pip install adafruit-blinka
```

### 4.4 Install Bleak
Install Bleak for Bluetooth Low Energy (BLE) communication:
```bash
pip install bleak
```

### 4.5 Install RPi.GPIO
Install RPi.GPIO for Raspberry Pi GPIO pin control:
```bash
pip install RPi.GPIO
```

---

## Step 5: Enable I2C on Raspberry Pi
Enable I2C using the Raspberry Pi configuration tool:

1. Open the configuration tool:
   ```bash
   sudo raspi-config
   ```

2. Navigate to **Interface Options > I2C** and enable it.

3. Reboot the system:
   ```bash
   sudo reboot
   ```

---

## Step 6: Test I2C
Check if I2C devices are detected:

1. Install I2C tools:
   ```bash
   sudo apt install i2c-tools -y
   ```

2. Run the following command to detect connected devices:
   ```bash
   i2cdetect -y 1
   ```

---

## Step 7: Check BLE Compatibility
Ensure BLE hardware is functioning by scanning for BLE devices:

1. Check for BLE devices:
   ```bash
   hcitool lescan
   ```

2. If `hcitool` is not found, install Bluetooth tools:
   ```bash
   sudo apt install bluetooth bluez -y
   ```

---

## Step 8: Make the Script Executable
Navigate to the directory where your script is located and make it executable:

1. Change file permissions:
   ```bash
   chmod +x /path/to/your/script.py
   ```

2. Verify the script is now executable:
   ```bash
   ls -l /path/to/your/script.py
   ```

   Ensure the file's permissions include `-rwxr-xr-x`.

---

## Step 9: Set Up the Script as a Startup Service

### 9.1 Create a Systemd Service File
Create a service file in `/etc/systemd/system`:
```bash
sudo nano /etc/systemd/system/robot.service
```

Add the following content, adjusting paths as necessary:
```plaintext
[Unit]
Description=Robotic Arm Startup Script
After=network.target

[Service]
ExecStart=/home/pi/myenv/bin/python3 /path/to/your/script.py
WorkingDirectory=/path/to/your/script
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```

### 9.2 Enable the Service
Enable the service to run at startup:
```bash
sudo systemctl enable robot.service
```

### 9.3 Start the Service
Start the service immediately:
```bash
sudo systemctl start robot.service
```

### 9.4 Verify the Service
Check the status to ensure it is running:
```bash
sudo systemctl status robot.service
```

---

## Step 10: Run the Script Manually (Optional)
You can manually run the script for testing:
```bash
source myenv/bin/activate
python3 /path/to/your/script.py
```

---

## Final Notes
If everything is configured correctly, the script will start automatically on reboot. To stop the service, use:
```bash
sudo systemctl stop robot.service
```

To disable the service from starting at boot:
```bash
sudo systemctl disable robot.service
