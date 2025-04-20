import rtde_receive
import rtde_control

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

def speed_moveL(cur_pos, target_pos, speed, gain):
    (TAKE TIME)
    delta_pos = [target_pos[i] - cur_pos[i] for i in range(len(target_pos))]
    while abs(sum(delta_pos)) > 0.01:
        speed = [max(-0.25, min(0.25, target_pose[i] - pose[i])) for i in range(6)] # Task-space P control with a maximum speed threshold and gain Kp = 1
        rtde_c.speedL(speed, 2, 0)
        pose = rtde_r.getActualTCPPose()
        delta_pose = [target_pose[i] - pose[i] for i in range(6)]
        time.sleep(0.1)
    return (TOTAL TIME)


time_to_grab = 3
z_target = 0
speed = 2
Kp = 1
delay = 1



class Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def ls(self):
        return [self.x, self.y, self.z]

class Velocity:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def ls(self):
        return [self.x, self.y, self.z]

class Bottle:
    def __init__(self, pos, vel, color):
        self.pos = pos
        self.vel = vel
        self.color = color
    
    def future_position(self, time_delay):
        return [self.pos.x + self.vel.x* time_delay,
                self.pos.y + self.vel.y*time_delay,
                self.pos.z,
                0,0,0]


drop_positions = {"blue": Position(-0.13163, -794.96,-319.50),
                  "green": Position(0.15800,-805.99,-319.50),
                  "red": Position(-440.52,-799.56,-319.50)}




while(True):
    bottle = get_target()

    target_pos = bottle.future_position(time_delay)
    cur_pos = rtde_r.getActualTCPPose()
    TIME_SPAN = speed_moveL(cur_pos, target_pos, speed, Kp)
    wait(time_to_grab - TIME_SPAN - delay)

    speedL(DOWN, speed)
    vacuum_on()

    while sense_bottle():
        cur_pos = rtde_r.getActualTCPPose()
        target_pos = target_logic(bottle.color)
        speed_moveL(cur_pos, target_pos, speed, Kp)
        vacuum_off()

    moveL(standby,0,0)

    

        


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