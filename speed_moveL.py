import rtde_receive
import rtde_control
import time
import serial

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

portName = '/dev/ttyACM1'

## Bin Positions (x, y, z, rx, ry, rz)
positions = {"yellow": [0.11874185587583301, -0.7891758751405292, 0.125397598041483435, -2.341806743327856, 2.0681131911584973, -0.03818426020594277],
             "blue": [-0.1525171470714167, -0.8030085393136451, 0.12541064764556142, -2.3417794446943234, 2.0680696902802116, -0.03818361678440171],
             "green": [-0.41902641634478266, -0.7951756441194379, 0.125368432165289723, -2.3417937824894195, 2.068177972269544, -0.038187638517122306],
             "colab": [0.36197288587652526, -0.7951662181724354, 0.125394826729175105, -2.3417869726414136, 2.0681194592170717, -0.038135188173779394],
             "neutral": [0.015466521999787283, -0.4899471595439902, 0.23258818072260725, -2.3417131655824277, 2.0681925358661806, -0.03821438995733724],
             "standby": [-0.29992314772168255, -0.4899471595439902, 0.05152892504111392,  -2.3417131655824277, 2.0681925358661806, -0.03821438995733724]

}

## Ideal Bounding Box (During gripping)
bounding_box = {"max": [-0.39074419281582623, -0.7766767888560709, 0.2364383950071206, -2.2125974178272223, -2.2092962831368443, 0.13271170247340863],
                "min": [0.41258058953437127, -0.15293392223319435, -0.02370832529093643, -2.4345706525566944, -1.9513767250305556, 0.05129300432761595]}

## Position where end effector is too close to base (make sure radius is always greater)
close = [0.17345416286195572, -0.22936740746653966, 0.02083577751346813, 2.650814714806161, 1.5789849693467959, -0.01100979896306741]

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
    return average_force < -0.15

## Check for contact with bottle.
def force_check(z_force):
    current_force = rtde_r.getActualTCPForce()
    return current_force[2] < z_force

## Check if current pose is within bounds.
def in_bounds(pose):
    return (pose[0] > bounding_box["max"][0] and pose[0] < bounding_box["min"][0] and
        pose[1] > bounding_box["max"][1] and pose[1] < bounding_box["min"][1] and
        pose[2] < bounding_box["max"][2] and pose[2] > bounding_box["min"][2] and
        vector_magnitude(close) <  vector_magnitude(pose) )

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

# # NEEDS TESTING
# def speed_followL(target_pose, speed):
#     pose = rtde_r.getActualTCPPose()
#     speed = [max(-0.25, min(0.25, target_pose[i] - pose[i])) for i in range(6)] # Task-space P control with a maximum speed threshold and gain Kp = 1
#     rtde_c.speedL(speed, 2, 0)
#     time.sleep(0.1)

## Throw bottle!
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

def autonomy(y_pos, color, time_to_grab):
    start_time = time.time()    
    pickup_time = 0.5
    speed_moveL(positions["standby"])

    target = pose.copy
    target[1] = y_pos[1]

    speed_moveL(target, vacuum=True)

    cur_time = time.time()
    while ( time_to_grab - (cur_time-start_time) - pickup_time > 0):
        cur_time = time.time()
        serial_port.write(b"on: True \n")
        time.sleep(0.1)

    target[2] = -0.012677585773136338
    rtde_c.zeroFtSensor()
    speed_moveL(target, force=True, vacuum=True)
    speed_moveL(positions["neutral"], vacuum=True)
    if not hold_check():
        return
    
    


if __name__ == '__main__':
    # PRINT ENDPOINT POSITION
    pose = rtde_r.getActualTCPPose()

    print(pose)

    serial_port = serial.Serial(port=portName, baudrate=115200, timeout=1, write_timeout=1)

    # speed_moveL(positions["standby"])
    # target = positions["standby"]
    # target[2] = -0.012677585773136338
    # speed_moveL(target, vacuum=True, force=True)

    target = pose.copy()
    target[2] = -0.012677585773136338

    start_time = time.time()
    while time.time() - start_time < 2:
        serial_port.write(b"on: True \n")
        time.sleep(0.25)
    rtde_c.zeroFtSensor()
    speed_moveL(target, force=True, vacuum=True)
    speed_moveL(positions["neutral"], vacuum=True)
    print(hold_check())

    # target = pose.copy()
    # target[0] += 0.1
    # speed_moveL(target, vacuum=True)
    
    # start_time = time.time()
    # while time.time() - start_time < 2:
    #     serial_port.write(b"on: True \n")
    #     time.sleep(0.25)
    # rtde_c.zeroFtSensor()
    # print("ZEROED")
    # while time.time() - start_time < 2:
    #     serial_port.write(b"on: True \n")
    #     time.sleep(0.25)
    # print(hold_check())
    # print(hold_check())
    # print(hold_check())
    # print(hold_check())
    # # throw()
    # serial_port.close()



    # # GO TO LOCATIONS.
    # speed_moveL(positions["standby"])
    # speed_moveL(positions["green"])
    # speed_moveL(positions["blue"])
    # speed_moveL(positions["yellow"])
    # speed_moveL(positions["colab"])
    # speed_moveL(positions["colab"])

    # pickuptime = 1
    # time limit

    # # AUTONOMY TEST
    # speed_moveL(positions["standby"])
    # target = pose.copy
    # target[1] = bottle.pos[1]

    # speed_moveL(target, vacuum=True)

    # while ( time_til_pickup - pickup_time > 0):
    #     time.sleep(0.2)


    # # CHECK BOX LIMITS
    # position = bounding_box["max"]
    # speed_moveL(position)

    # target = position
    # target[0] = bounding_box["min"][0]

    # speed_moveL(target)
    # position = target
    # target[1] = bounding_box["min"][1]

    # speed_moveL(target)
    # position = target
    # target[2] = bounding_box["min"][2]

    # speed_moveL(target)