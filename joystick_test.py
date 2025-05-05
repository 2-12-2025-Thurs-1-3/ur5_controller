import rtde_receive
import rtde_control
import time
import serial
import Final_CoM as cv

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")
portName = '/dev/ttyACM0'

ORANGE_BOTTLE_COUNT = 0
VEL_STEP = 0.1
ANG_STEP = 0.2

## SPEED 1.5

# Bin Positions (x, y, z, rx, ry, rz)
positions = {"yellow": [0.11874185587583301, -0.7891758751405292, 0.125397598041483435, -2.341806743327856, 2.0681131911584973, -0.03818426020594277],
             "blue": [-0.1525171470714167, -0.8030085393136451, 0.12541064764556142, -2.3417794446943234, 2.0680696902802116, -0.03818361678440171],
             "green": [-0.3306282131121385, -0.8011531910300942, 0.2566662052170944, 2.020934453549782, -2.4038836258835543, 0.08104260776482829],
             "collab": [0.36197288587652526, -0.7951662181724354, 0.125394826729175105, -2.3417869726414136, 2.0681194592170717, -0.038135188173779394],
             "neutral": [0.015466521999787283, -0.4899471595439902, 0.23258818072260725, -2.3417131655824277, 2.0681925358661806, -0.03821438995733724],
             "standby": [-0.29992314772168255, -0.4899471595439902, 0.05152892504111392,  -2.3417131655824277, 2.0681925358661806, -0.03821438995733724]
}

# Check xyz magnitude of vector
def vector_magnitude(delta_pose, joint = False):
    if joint:
        return (sum([d*d for d in delta_pose[:]]))**0.5  # Total joint position error.
    return (sum([d*d for d in delta_pose[:3]]))**0.5  # Only XYZ translation error

# True if bottle is held
def hold_check():
    start_time = time.time()
    i = 0
    total_force = 0
    while time.time() - start_time < 2:
        i+=1
        total_force += rtde_r.getActualTCPForce()[2]
        serial_port.write(b"on: True \n")
    average_force = total_force/i
    print(average_force)
    return average_force < -0.15

# Check for endpoint force exceeding "z_force"
def force_check(z_force):
    current_force = rtde_r.getActualTCPForce()
    return current_force[2] < z_force

# Go to target (x,y,z) position.
def moveL(target_pose, force=False, vacuum=False):
    move_duration = 0.1
    acceleration = 2.0
    tolerance = 0.005  # 5 mm

    pose = rtde_r.getActualTCPPose()
    delta_pose = [target_pose[i] - pose[i] for i in range(6)]

    while vector_magnitude(delta_pose) > tolerance and (force_check(10.0) or not force):

        if vacuum:
            serial_port.write(b"on: True \n")

        pose = rtde_r.getActualTCPPose()
        delta_pose = [2*(target_pose[i] - pose[i]) for i in range(6)]

        # Create speed command: Only XYZ are moving, rotations stay zero
        speed = [0, 0, 0, 0, 0, 0]
        for i in range(3):  # Only for x, y, z
            speed[i] = max(-0.25, min(0.25, delta_pose[i]))  # Clip 
        

        rtde_c.speedL(speed, acceleration, move_duration)
        time.sleep(move_duration)

    # Stop the robot
    rtde_c.speedL([0, 0, 0, 0, 0, 0], acceleration, 0)

# Go to target (rx,ry,rz) position.
def moveJ(target_pose, vacuum=False):
    move_duration = 0.1
    acceleration = 2.0
    tolerance = 0.005

    pose = rtde_r.getActualQ()
    delta_pose = [target_pose[i] - pose[i] for i in range(6)]

    while vector_magnitude(delta_pose, joint=True) > tolerance:

        if vacuum:
            serial_port.write(b"on: True \n")

        pose = rtde_r.getActualQ()
        delta_pose = [2*(target_pose[i] - pose[i]) for i in range(6)]

        speed = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            speed[i] = max(-0.25, min(0.25, delta_pose[i]))  # Clip 
        

        rtde_c.speedJ(speed, acceleration, move_duration)
        time.sleep(move_duration)

    # Stop the robot
    rtde_c.speedJ([0, 0, 0, 0, 0, 0], acceleration, 0)

# Given bottle color, where bottle should go.
def bin_logic(color):
    global ORANGE_BOTTLE_COUNT
    if color == 0:
        return positions["yellow"]
    if color == 1:
        return positions["blue"]
    if color == 2:
        ORANGE_BOTTLE_COUNT += 1
        if ORANGE_BOTTLE_COUNT == 2:
            return positions["collab"]
        else:
            smh_head()
            return False
    if color == 3:
        return positions["green"]

# Throw or place bottle.
def throw(color):
    acceleration = 4.0
    if color == 2:
        moveL(positions["collab"], vacuum=True)
        start_time = time.time()
        while time.time() - start_time < 1:
            serial_port.write(b"on: False \n")
            time.sleep(0.25)    
    else:
        moveL(bin_logic(color),vacuum=True)
        dunk()

    rtde_c.speedL([0, 0, 0, 0, 0, 0], acceleration, 0)  # Stop immediately

def dunk():
    move_duration = 1
    acceleration = 4

    original = rtde_r.getActualQ()

    J = [0,0,0,-1,0,0]
    rtde_c.speedJ(J, acceleration, move_duration)
    time.sleep(move_duration/2)
    serial_port.write(b"on: False \n")
    time.sleep(move_duration/2)
    rtde_c.speedJ([0,0,0,0,0,0], acceleration, 0)
    time.sleep(1)

    moveJ(original)
    return

def smh_head():
    original = rtde_r.getActualQ()
    move_duration = 0.5
    acceleration = 2
    J = [0,0,0,0,-1,0]

    rtde_c.speedJ(J, acceleration, move_duration)
    time.sleep(move_duration)
    rtde_c.speedJ([0,0,0,0,0,0], acceleration, 0)
    time.sleep(move_duration)
    for j in range(2):
        bru = -1 if j%2==0 else 1
        rtde_c.speedJ([bru*_ for _ in J], acceleration, 2*move_duration)
        time.sleep(move_duration)
        rtde_c.speedJ([0,0,0,0,0,0], acceleration, 0)
        time.sleep(move_duration)
    rtde_c.speedJ([-_ for _ in J], acceleration, move_duration)
    time.sleep(move_duration)
    rtde_c.speedJ([0,0,0,0,0,0], acceleration, 0)
    time.sleep(move_duration)
    moveJ(original)
    return

# Grab and place bottle in bin.
def autonomy(time_to_grab, color, y_pos):
    
    start_time = time.time()    
    pickup_time = 0

    if y_pos > -0.17311455981260457:
        y_pos = -.1731145598126

    grab_target = positions["standby"].copy()
    grab_target[1] = y_pos

    target_bin = bin_logic(color)
    if not target_bin:
        return

    moveL(grab_target, vacuum=True)

    while ( time_to_grab - (time.time()-start_time) - pickup_time > 0):
        serial_port.write(b"on: True \n")
        time.sleep(0.01)

    grab_target[2] = -0.04286568666613467
    rtde_c.zeroFtSensor()
    moveL(grab_target, force=True, vacuum=True)
    time.sleep(0.25)
    moveL(positions["neutral"], vacuum=True)

    start_time = time.time()
    while time.time() - start_time < 2:
        if not joystickStop():
            return True  
        time.sleep(0.25)
    
    # if not hold_check():
    #     moveL(positions["neutral"], vacuum=True)
    #     return 
    
    throw(color)
    return

# Runs autonomy in a loop.   
def autonomy_loop():
    rtde_c.moveL(positions["neutral"])
    while True:
        data = cv.detect()
        print(data)
        if data is not None:
            if autonomy(*data):
                return True
            moveL(positions["neutral"])

# JOYSTICK LAND
def joystickConvert(data):
    new_data = [0,0,0,0,0,0]
    new_data[:2] = data[:2]
    new_data[2] = data[2] - data[3]
    # new_data[5] = data[4] - data[5] # REPLACE WITH NEW BUTTON FUNCTIONS (Blue button not needed, since it communicates to vacuum directly)
    
    linear = [val * VEL_STEP for val in new_data[0:3]]
    angular = [val * ANG_STEP for val in new_data[3:]]
    new_data = linear + angular         # Get velocities
    return new_data

def joystickStop():
    try:
        esp32_output = serial_port.readline().decode('utf-8').strip()
        vals = esp32_output.split(",")
        data = [float(val.strip()) for val in vals[:-1]]
        if len(data) < 6:
            return False
        if not (data[2] or data[3] or data[4] or data[5]):
            return data
    except:
        pass
    return False

def joystickLoop():
    try:
        if serial_port.in_waiting > 0:
            data = joystickStop()
            print(data)
            if not data:  # Stop condition
                return True
            convert = joystickConvert(data)
            rtde_c.speedL(convert)
            serial_port.reset_input_buffer()
        else:
            time.sleep(0.00101)  # Allow loop to rest

    except Exception as e:
        print(1)

    return False  # Still running unless joystickStop triggered

if __name__ == '__main__':
    #if rtde_run == True: rtde_c.moveL(init_position)
    # Connect to the ESP32. You can use PlatformIO to find its COM port. Remember that the COM port is different in BOOT mode than when code is running!
    serial_port = serial.Serial(port=portName, baudrate=115200, timeout=1, write_timeout=1)

    changeMode = False
    joystickMode = True
    while True:
        if joystickMode:
            changeMode = joystickLoop()
            print("JOYSTICK")
        else:
            changeMode = autonomy_loop()
        if changeMode:
            joystickMode = not joystickMode
            changeMode = False