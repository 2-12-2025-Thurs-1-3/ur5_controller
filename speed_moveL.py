import rtde_receive
import rtde_control
import time
import serial

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

portName = '/dev/ttyACM1'

positions = {"yellow": [0.11874185587583301, -0.7891758751405292, -0.025397598041483435, -2.341806743327856, 2.0681131911584973, -0.03818426020594277],
             "blue": [-0.1525171470714167, -0.8030085393136451, -0.02541064764556142, -2.3417794446943234, 2.0680696902802116, -0.03818361678440171],
             "green": [-0.41902641634478266, -0.7951756441194379, -0.025368432165289723, -2.3417937824894195, 2.068177972269544, -0.038187638517122306],
             "colab": [0.36197288587652526, -0.7951662181724354, -0.025394826729175105, -2.3417869726414136, 2.0681194592170717, -0.038135188173779394],
             "neutral": [0.015466521999787283, -0.4899471595439902, 0.13258818072260725, -2.3417131655824277, 2.0681925358661806, -0.03821438995733724]}


bounding_box = {"max": [-0.39074419281582623, -0.7766767888560709, 0.2364383950071206, -2.2125974178272223, -2.2092962831368443, 0.13271170247340863],
                "min": [0.41258058953437127, -0.15293392223319435, -0.02370832529093643, -2.4345706525566944, -1.9513767250305556, 0.05129300432761595]}

close = [0.17345416286195572, -0.22936740746653966, 0.02083577751346813, 2.650814714806161, 1.5789849693467959, -0.01100979896306741]

def in_bounds(pose):
    return (pose[0] > bounding_box["max"][0] and pose[0] < bounding_box["min"][0] and
        pose[1] > bounding_box["max"][1] and pose[1] < bounding_box["min"][1] and
        pose[2] < bounding_box["max"][2] and pose[2] > bounding_box["min"][2] and
        (sum([ i**2 for i in close[0:3]]))**(0.5) <  (sum([ i**2 for i in pose[0:3]]))**(0.5) )

def force_check(z_force):
    current_force = rtde_r.getActualTCPForce()
    return current_force[2] < z_force

def pose_error(delta_pose):
    return (sum([d*d for d in delta_pose[:3]]))**0.5  # Only XYZ translation error
def speed_moveL(target_pose, bounds=False, force=False):
    move_duration = 0.1
    acceleration = 2.0
    tolerance = 0.005  # 5 mm

    pose = rtde_r.getActualTCPPose()
    delta_pose = [target_pose[i] - pose[i] for i in range(6)]

    while pose_error(delta_pose) > tolerance and (in_bounds(pose) or not bounds) and (force_check(10.0) or not force):
        pose = rtde_r.getActualTCPPose()
        delta_pose = [target_pose[i] - pose[i] for i in range(6)]

        # Create speed command: Only XYZ are moving, rotations stay zero
        speed = [0, 0, 0, 0, 0, 0]
        for i in range(3):  # Only for x, y, z
            speed[i] = max(-0.25, min(0.25, delta_pose[i]))  # Clip speed

        rtde_c.speedL(speed, acceleration, move_duration)
        time.sleep(move_duration)

    # Stop the robot
    rtde_c.speedL([0, 0, 0, 0, 0, 0], acceleration, 0)


## NEEDS TESTING
def speed_followL(target_pose, speed):
    pose = rtde_r.getActualTCPPose()
    speed = [max(-0.25, min(0.25, target_pose[i] - pose[i])) for i in range(6)] # Task-space P control with a maximum speed threshold and gain Kp = 1
    rtde_c.speedL(speed, 2, 0)
    time.sleep(0.1)



def throw():
    speed = [0, -1, 0, 0, 0, 0]
    acceleration = 2.0
    move_duration = 0.02  # Move for 20 milliseconds each command

    for i in range(20):
        print(i)
        if i > 18:
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


if __name__ == '__main__':
    # PRINT ENDPOINT POSITION
    pose = rtde_r.getActualTCPPose()
    print(pose)

    # target = pose
    # target[0] += 0.1
    # speed_moveL(target)

    serial_port = serial.Serial(port=portName, baudrate=115200, timeout=1, write_timeout=1)


    start_time = time.time()
    while time.time() - start_time < 10:
        serial_port.write(b"on: True \n")
        time.sleep(0.25)
    throw()
    serial_port.close()



    # # GO TO LOCATIONS.
    # speed_moveL(positions["neutral"])
    # speed_moveL(positions["green"])
    # speed_moveL(positions["blue"])
    # speed_moveL(positions["yellow"])
    # speed_moveL(positions["colab"])
    # speed_moveL(positions["colab"])

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

# time_to_grab = 3
# z_target = 0
# speed = 2
# Kp = 1
# delay = 1



# class Position:
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z
    
#     def ls(self):
#         return [self.x, self.y, self.z]

# class Velocity:
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z

#     def ls(self):
#         return [self.x, self.y, self.z]

# class Bottle:
#     def __init__(self, pos, vel, color):
#         self.pos = pos
#         self.vel = vel
#         self.color = color
    
#     def future_position(self, time_delay):
#         return [self.pos.x + self.vel.x* time_delay,
#                 self.pos.y + self.vel.y*time_delay,
#                 self.pos.z,
#                 0,0,0]


# drop_positions = {"blue": Position(-0.13163, -794.96,-319.50),
#                   "green": Position(0.15800,-805.99,-319.50),
#                   "red": Position(-440.52,-799.56,-319.50)}




# while(True):
#     bottle = get_target()

#     target_pos = bottle.future_position(time_delay)
#     cur_pos = rtde_r.getActualTCPPose()
#     TIME_SPAN = speed_moveL(cur_pos, target_pos, speed, Kp)
#     wait(time_to_grab - TIME_SPAN - delay)

#     speedL(DOWN, speed)
#     vacuum_on()

#     while sense_bottle():
#         cur_pos = rtde_r.getActualTCPPose()
#         target_pos = target_logic(bottle.color)
#         speed_moveL(cur_pos, target_pos, speed, Kp)
#         vacuum_off()

#     moveL(standby,0,0)

    

        


"""
- Get bottle data (position, velocity, color)
- Calculate the intersection point
- Move to intersection point
- Activate vacuum
- Lift
- Find if bottle is attached
if attached:
    find color
    move to endpoint
    release vacuum
    move to standby

if not attached:
    move to standby

"""