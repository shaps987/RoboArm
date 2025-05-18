Installing the usmbus Library on Raspberry Pi Pico
--------------------------------------------------

This document details the steps to download and install the `usmbus` library on a Raspberry Pi Pico. The library enables I2C communication for devices like the ADS7830 ADC.

* * * * *

Prerequisites
-------------

-   Raspberry Pi Pico with MicroPython installed.
-   MicroPython REPL accessible via `mpremote`.
-   A development computer with Python installed and access to a terminal.

* * * * *

Steps
-----

### 1\. Download the usmbus Library

1.  Go to the [usmbus GitHub repository](https://github.com/mcauser/micropython-smbus) on your computer.
2.  Download the repository as a ZIP file:
    -   Click the green "Code" button and select **Download ZIP**.
3.  Extract the ZIP file to access the library source files.

Or just drag the lib folder with the neccessary package included in this github repository into your Pico via an IDE like Thonny. From there, you can import usumbus as normal.

* * * * *

### 2\. Copy the Library Files to the Pico

1.  Connect your Raspberry Pi Pico to your computer via USB.
2.  Confirm that your Pico is detected:

Bash

```
mpremote connect list

```

Note the COM port or device path of your Pico.

1.  Create a `/lib` directory on the Pico (if it doesn't already exist):

Bash

```
mpremote connect <COM_PORT> fs mkdir /lib

```

Replace `<COM_PORT>` with the detected port or device path.

1.  Copy the `usmbus` library folder to the Pico:

Bash

```
mpremote connect <COM_PORT> fs cp -r path/to/micropython-smbus/usmbus /lib/

```

Replace `path/to/micropython-smbus` with the path where you extracted the library.

### 3\. Verify Installation

Open the Pico's REPL:

Bash

```
mpremote connect <COM_PORT> repl

```

Test importing the library:

Python

```
>>> from usmbus import SMBus
>>> bus = SMBus(0)
>>> print(bus)

```

If there are no errors, the library is successfully installed.

* * * * *

Automating Script Execution on Startup
--------------------------------------

1.  Rename your MicroPython script to `main.py`:

Bash

```
mv your_script.py main.py

```

1.  Copy the script to the Pico:

Bash

```
mpremote connect <COM_PORT> fs cp path/to/main.py /

```

The script will now execute automatically when the Pico is powered on.

* * * * *

Troubleshooting
---------------

**Error: Module not found**

-   Ensure the library folder is copied to the `/lib` directory on the Pico.
-   Verify the Pico is running MicroPython.

**Connection Issues**

-   Check USB connections and ensure the correct COM port is used.
-   Run `mpremote connect list` to verify the Pico is detected.