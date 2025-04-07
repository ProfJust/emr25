from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from robotiq_gripper_control import RobotiqGripper
import time

UR3_IP = "127.0.0.1"

rtde_r = RTDEReceiveInterface(UR3_IP)
rtde_c = RTDEControlInterface(UR3_IP)
gripper = RobotiqGripper(rtde_c)

# Roboter bewegen
#print("🤖 moveL...")
#rtde_c.moveL([-0.3, -0.7, 0.4, 0.0, 3.2, 0.1])

# Gripper Aktionen
#gripper.activate()
#gripper.set_force(50)
#gripper.set_speed(100)
gripper.close()
time.sleep(1)
gripper.open()
time.sleep(1)
gripper.close()
time.sleep(1)
#gripper.move(10)  # mm

# Abschluss
rtde_c.disconnect()
rtde_r.disconnect()
