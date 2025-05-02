import rtde_receive
import rtde_control
import time
import serial
import Final_CoM as cv
import robot as sp

portName = "/dev/ttyACM0"
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")


# Stays in initial position. If needed place bottle after printing "ZEROED".
# hold_check prints True when bottle is attached, False otherwise.
def test_hold():
    start_time = time.time()
    while time.time() - start_time < 1:
        serial_port.write(b"on: True \n")
        time.sleep(0.25)
    rtde_c.zeroFtSensor()
    print("ZEROED")
    while time.time() - start_time < 7:
        serial_port.write(b"on: True \n")
        time.sleep(0.25)
    print(sp.hold_check())
    print(sp.hold_check())
    print(sp.hold_check())
    print(sp.hold_check())

# Move to set positions
def test_move():
    sp.speed_moveL(sp.positions["standby"])
    sp.speed_moveL(sp.positions["green"])
    sp.speed_moveL(sp.positions["blue"])
    sp.speed_moveL(sp.positions["yellow"])
    sp.speed_moveL(sp.positions["colab"])
    sp.speed_moveL(sp.positions["colab"])

def test_drop():
    pose = rtde_r.getActualTCPPose()
    target = pose.copy()
    target[2] = -0.012677585773136338

    start_time = time.time()
    while time.time() - start_time < 2:
        serial_port.write(b"on: True \n")
        time.sleep(0.25)
    rtde_c.zeroFtSensor()
    sp.speed_moveL(target, force=True, vacuum=True)
    sp.speed_moveL(sp.positions["neutral"], vacuum=True)
    print(sp.hold_check())
    sp.speed_moveL(sp.positions["green"], vacuum=True)
    serial_port.write(b"on: False \n")

if __name__ == '__main__':
    serial_port = serial.Serial(port=portName, baudrate=115200, timeout=1, write_timeout=1)
    # test_move()
    # test_hold()
    # test_drop()
    serial_port.close()