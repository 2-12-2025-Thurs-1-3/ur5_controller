import rtde_receive
import rtde_control
import time
import serial
import Final_CoM as cv

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")
portName = '/dev/ttyACM0'


# GOALS FOR TODAY
# 1. Integrating throwing with autonomy.
# 2. Testing throwing into yellow bin. (eliminate for collab and green)
# 3. Nodding head when orange bottle. 
# 4. Force tuning clear bottles. 


ORANGE_BOTTLE_COUNT = 0

# Bin Positions (x, y, z, rx, ry, rz)
positions = {"yellow": [0.11874185587583301, -0.7891758751405292, 0.125397598041483435, -2.341806743327856, 2.0681131911584973, -0.03818426020594277],
             "blue": [-0.1525171470714167, -0.8030085393136451, 0.12541064764556142, -2.3417794446943234, 2.0680696902802116, -0.03818361678440171],
             "green": [-0.41902641634478266, -0.7951756441194379, 0.125368432165289723, -2.3417937824894195, 2.068177972269544, -0.038187638517122306],
             "collab": [0.36197288587652526, -0.7951662181724354, 0.125394826729175105, -2.3417869726414136, 2.0681194592170717, -0.038135188173779394],
             "neutral": [0.015466521999787283, -0.4899471595439902, 0.23258818072260725, -2.3417131655824277, 2.0681925358661806, -0.03821438995733724],
             "standby": [-0.29992314772168255, -0.4899471595439902, 0.05152892504111392,  -2.3417131655824277, 2.0681925358661806, -0.03821438995733724]
}
throw_pos = {"green":[-0.31230473033609324, -0.822971401718836, 0.16310178899080682, 1.7926729777558967, -2.573651269089924, 0.12884544780131027]
}

# Check xyz magnitude of vector
def vector_magnitude(delta_pose, joint = False):
    if joint:
        return (sum([d*d for d in delta_pose[:]]))**0.5
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
        if ORANGE_BOTTLE_COUNT == 4:
            return positions["collab"]
        else:
            smh_head()
            return False
    if color == 3:
        return positions["green"]

# TEST CODE
def throw(color):
    speed = [0, -1, 0, 0, 0, 0]
    acceleration = 4.0
    move_duration = 0.02  # Move for 20 milliseconds each command
    
    if color == 10:
        origin = positions["neutral"]
        origin[0] = bin_logic(color)[0]
        moveL(origin, vacuum=True)
        
        
        for i in range(16):
            print(i)
            current_pose = rtde_r.getActualTCPPose()
            if (current_pose[1] < -0.4):
                serial_port.write(b"on: False \n")
            else:
                serial_port.write(b"on: True \n")
            rtde_c.speedL(speed, acceleration, move_duration)
            time.sleep(move_duration)  # Important! Let the move happen
    else:
        moveL(bin_logic(color),vacuum=True)
        dunk()

    print("STOP")
    rtde_c.speedL([0, 0, 0, 0, 0, 0], acceleration, 0)  # Stop immediately

# Grab and place bottle in bin.
def autonomy(time_to_grab, color, y_pos):
    
    start_time = time.time()    
    pickup_time = 0

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
    
    
    throw(color)
    #moveL(target_bin, vacuum=True)
    #serial_port.write(b"on: False \n")
    #time.sleep(1)
    #moveL(positions["neutral"])
    return

# Runs autonomy in a loop.   
def autonomy_loop():
    rtde_c.moveL(positions["neutral"])
    while True:
        data = cv.detect()
        print(data)
        if data is not None:
            autonomy(*data)
            moveL(positions["neutral"], vacuum=True)

def dunk():
    move_duration = 0.5
    acceleration = 4

    original = rtde_r.getActualQ()

    J = [0,0,0,-1,0,0]
    rtde_c.speedJ(J, acceleration, move_duration)
    serial_port.write(b"on: False \n")
    time.sleep(move_duration)
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

def tilt_head(shake = False):
    move_duration = 0.5
    acceleration = 2
    if shake:
        J = [0,0,0,0,-1,0]
    else:
        J = [0,0,0,-1,0,0]

    rtde_c.speedJ(J, acceleration, move_duration)
    time.sleep(move_duration)
    rtde_c.speedJ([0,0,0,0,0,0], acceleration, 0)
    time.sleep(move_duration)
    if shake:
        for j in range(2):
            bru = -2 if j%2==0 else 2
            rtde_c.speedJ([bru*_ for _ in J], acceleration, move_duration)
            time.sleep(move_duration)
            rtde_c.speedJ([0,0,0,0,0,0], acceleration, 0)
            time.sleep(move_duration)
    rtde_c.speedJ([-_ for _ in J], acceleration, move_duration)
    time.sleep(move_duration)
    rtde_c.speedJ([0,0,0,0,0,0], acceleration, 0)
    time.sleep(move_duration)

if __name__ == '__main__':

    serial_port = serial.Serial(port=portName, baudrate=115200, timeout=1, write_timeout=1)
    autonomy_loop()
    # origin = positions["neutral"]
    # origin[0] = positions["blue"][0]
    # start_time = time.time()
    # while time.time() - start_time < 5:
    #     serial_port.write(b"on: True \n")
    #     time.sleep(0.25)

    # moveL(origin, vacuum=True)
    # throw()

    # print(rtde_r.getActualTCPPose())
    # print(rtde_r.getActualQ())
    # start_time = time.time()
    # while time.time() - start_time < 5:
    #     serial_port.write(b"on: True \n")
    #     time.sleep(0.25)
    # throw(1)
    serial_port.close()


    #THROW POSITION FOR GREEN
    #[-0.2536105786193372, -0.8345584232856752, 0.1676398144363905, 2.2346443562811844, -2.1900617416901316, -0.0038242945823081156]

    #Joint angles at throw position
    # [-5.160933319722311, -0.4703729909709473, 0.7006576696978968, -1.810779710809225, -1.5812862555133265, -2.0395692030536097]

    #Joint angles after moving joint
    # [-5.161161247883932, -0.4700372976115723, 0.7014721075641077, -2.515862127343649, -1.5813220183001917, -2.039593521748678]