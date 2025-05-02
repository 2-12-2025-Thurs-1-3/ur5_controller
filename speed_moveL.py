import rtde_receive
import rtde_control
import time
import serial
import Final_CoM as cv

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

portName = '/dev/ttyACM0'

ORANGE_BOTTLE_COUNT = 0

## Bin Positions (x, y, z, rx, ry, rz)
positions = {"yellow": [0.11874185587583301, -0.7891758751405292, 0.125397598041483435, -2.341806743327856, 2.0681131911584973, -0.03818426020594277],
             "blue": [-0.1525171470714167, -0.8030085393136451, 0.12541064764556142, -2.3417794446943234, 2.0680696902802116, -0.03818361678440171],
             "green": [-0.41902641634478266, -0.7951756441194379, 0.125368432165289723, -2.3417937824894195, 2.068177972269544, -0.038187638517122306],
             "collab": [0.36197288587652526, -0.7951662181724354, 0.125394826729175105, -2.3417869726414136, 2.0681194592170717, -0.038135188173779394],
             "neutral": [0.015466521999787283, -0.4899471595439902, 0.23258818072260725, -2.3417131655824277, 2.0681925358661806, -0.03821438995733724],
             "standby": [-0.29992314772168255, -0.4899471595439902, 0.05152892504111392,  -2.3417131655824277, 2.0681925358661806, -0.03821438995733724]

}

## Check xyz magnitude of vector
def vector_magnitude(delta_pose):
    return (sum([d*d for d in delta_pose[:3]]))**0.5  # Only XYZ translation error

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

## Check for contact with bottle.
def force_check(z_force):
    current_force = rtde_r.getActualTCPForce()
    return current_force[2] < z_force

# Go to target (x,y,z) position.
def speed_moveL(target_pose, bounds=False, force=False, vacuum=False):
    move_duration = 0.1
    acceleration = 2.0
    tolerance = 0.005  # 5 mm

    pose = rtde_r.getActualTCPPose()
    delta_pose = [target_pose[i] - pose[i] for i in range(6)]

    while vector_magnitude(delta_pose) > tolerance and ((in_bounds(pose) or not bounds) and (force_check(10.0) or not force)):

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
            return False
    if color == 3:
        return positions["green"]

def throw():
    speed = [0, -1, 0, 0, 0, 0]
    acceleration = 4.0
    move_duration = 0.02  # Move for 20 milliseconds each command

    for i in range(15):
        print(i)
        if i > 12:
            current_pose = rtde_r.getActualTCPPose()
            if (current_pose[1] < -0.8696310153479258,):
                serial_port.write(b"on: False \n")
            else:
                serial_port.write(b"on: True \n")
        else:
            serial_port.write(b"on: True \n")
        rtde_c.speedL(speed, acceleration, move_duration)
        time.sleep(move_duration)  # Important! Let the move happen

    print("STOP")
    rtde_c.speedL([0, 0, 0, 0, 0, 0], acceleration, 0)  # Stop immediately

def autonomy(time_to_grab, color, y_pos):
    
    start_time = time.time()    
    pickup_time = 0

    grab_target = positions["standby"].copy()
    grab_target[1] = y_pos

    target_bin = bin_logic(color)
    if not target_bin:
        return

    speed_moveL(grab_target, vacuum=True)

    while ( time_to_grab - (time.time()-start_time) - pickup_time > 0):
        serial_port.write(b"on: True \n")
        time.sleep(0.01)


    grab_target[2] = -0.04286568666613467
    rtde_c.zeroFtSensor()
    speed_moveL(grab_target, force=True, vacuum=True)
    time.sleep(0.25)
    speed_moveL(positions["neutral"], vacuum=True)

    # time.sleep(1)
    # if not hold_check():
    #     return
    
    speed_moveL(target_bin, vacuum=True)
    serial_port.write(b"on: False \n")
    time.sleep(1)
    speed_moveL(positions["neutral"])
    return
    
def autonomy_loop():
    rtde_c.moveL(positions["neutral"])
    while True:
        data = cv.detect()
        print(data)
        if data is not None:
            autonomy(*data)
            speed_moveL(positions["neutral"], vacuum=True)

if __name__ == '__main__':
    serial_port = serial.Serial(port=portName, baudrate=115200, timeout=1, write_timeout=1)
    autonomy_loop()
    serial_port.close()