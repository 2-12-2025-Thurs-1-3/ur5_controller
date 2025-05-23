from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import serial # Pyserial. Install as "pip install pyserial"
from pynput import keyboard
import threading

"""
JOYSTICK_CONTROL:

Joystick (using Serial Monitor) / Keyboard control for UR5 arm. 

Warnings:
    General:
        - Replace controlMODE with your desired mode before running.
        - Watch values for VEL_STEP and ANG_STEP
        - Always use rtde_run = False before testing with UR5.
        - set rtde_run to True if you are connected to UR5.

    Joystick:
        - In order to use Joystick Control, you must have a Serial Monitor running on a different window.
        - To read the serial monitor, you MUST replace portName with the ESP32's port.

    Keyboard:
        - Keybinds:     AD (-X, +X), WS (+Y, -Y), ZX (+Z, -Z), EQ (+YAW, -YAW)
        - To turn vacuum on/off, you must be running "j_vacuum.cpp" on the joystick, and open the joystick's
          serial monitor.  
        - Tip: Click into the terminal before using the keyboard so you don't edit the code.
"""

portName = '/dev/ttyACM1'
rtde_run = True


VEL_STEP = 0.1
ANG_STEP = 0.2

#init_position = [0.155, -0.508, -0.113, 3.14, 0, 0] # DECIDE LATER WITH A TA

if rtde_run:
    rtde_c = RTDEControl("192.168.1.103")
    rtde_r = RTDEReceive("192.168.1.103")

def joystickConvert(data):
    new_data = [0,0,0,0,0,0]
    new_data[:2] = data[:2]
    new_data[2] = data[2] - data[3]
    # new_data[5] = data[4] - data[5] # REPLACE WITH NEW BUTTON FUNCTIONS (Blue button not needed, since it communicates to vacuum directly)
    linear = [val * VEL_STEP for val in new_data[0:3]]
    angular = [val * ANG_STEP for val in new_data[3:]]
    new_data = linear + angular         # Get velocities
    return new_data

def joystickStop(data):
    return not (data[2] or data[3] or data[4] or data[5])

def joystickLoop():
    try:
        if serial_port.in_waiting > 0:
            esp32_output = str(serial_port.readline())
            vals = esp32_output.split(",")

            data = [float(val.strip("b' ")) for val in vals[:-1]]
            
            if joystickStop(data):  # Stop condition
                return False

            data = joystickConvert(data)
            print(data)

            if rtde_run:
                rtde_c.speedL(data)

            serial_port.reset_input_buffer()
        else:
            time.sleep(0.00101)  # Allow loop to rest

    except Exception as e:
        print(1)

    return True  # Still running unless joystickStop triggered


## READ MULTIPLE KEYS AT ONCE.
def keyboardLoop(rtde_c, rtde_run, VEL_STEP, ANG_STEP):
    velocity_cmd = [0, 0, 0, 0, 0, 0]
    running = [True]  # Using a list for mutability inside closures

    def on_press(key):
        try:
            if key.char != "e":
                serial_port.write(b"on: False \n")

            if key.char == 'w':
                velocity_cmd[1] = VEL_STEP  # +Y
            elif key.char == 's':
                velocity_cmd[1] = -VEL_STEP  # -Y
            if key.char == 'a':
                velocity_cmd[0] = -VEL_STEP  # -X
            elif key.char == 'd':
                velocity_cmd[0] = VEL_STEP  # +X
            if key.char == 'e':
                serial_port.write(b"on: True \n")
            # elif key.char == 'e':
            #     velocity_cmd[5] = ANG_STEP  # +yaw
            # elif key.char == 'q':
            #     velocity_cmd[5] = -ANG_STEP  # -yaw
            if key.char == 'z':
                velocity_cmd[2] = VEL_STEP  # +Z
            elif key.char == 'x':
                velocity_cmd[2] = -VEL_STEP  # -Z
        except AttributeError:
            pass

    def on_release(key):
        if key == keyboard.Key.esc:
            running[0] = False
            return False  # Stops the listener
        try:
            if key.char in 'wasdqxez':
                for i in range(6):
                    velocity_cmd[i] = 0
        except AttributeError:
            pass

    def keyboard_control_loop():
        rate = 0.01  # seconds
        while running[0]:
            print(velocity_cmd)
            if rtde_run:
                rtde_c.speedL(velocity_cmd, 0.5, rate)  # acceleration, time
            time.sleep(rate)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    keyboard_thread = threading.Thread(target=keyboard_control_loop)
    keyboard_thread.start()

    listener.join()
    keyboard_thread.join()


if __name__ == '__main__':
    running = True
    #if rtde_run == True: rtde_c.moveL(init_position)
    # Connect to the ESP32. You can use PlatformIO to find its COM port. Remember that the COM port is different in BOOT mode than when code is running!
    serial_port = serial.Serial(port=portName, baudrate=115200, timeout=1, write_timeout=1)
    j = 0
    while (j< 100000 and running):
        running = joystickLoop()
        j+=1
    serial_port.close()



# Stop the RTDE control script
if rtde_run: rtde_c.stopScript()