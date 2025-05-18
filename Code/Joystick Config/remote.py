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