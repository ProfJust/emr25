from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from robotiq_gripper_control import RobotiqGripper
import time

UR3_IP = "127.0.0.1"

rtde_r = RTDEReceiveInterface(UR3_IP)
rtde_c = RTDEControlInterface(UR3_IP)
gripper = RobotiqGripper(rtde_c)



# Gripper Aktionen
gripper.activate()
gripper.set_force(50)
gripper.set_speed(100)
gripper.close()

# Roboter bewegen
input("Roboter startet Bewegung nach Eingabe beliebiger Taste")
rtde_c.moveL([-0.300, -0.700 , 0.4,  0.0, 2.204, 0.1], 0.5, 0.3)
print("🤖 moveL...")
# rtde_c.moveL([-0.3, -0.7, 0.4, 0.0, 3.2, 0.1])
gripper.open()
#gripper.move_and_wait_for_pos(255, 255, 255)
actual_q = rtde_r.getActualQ() # in radian
print("Aktueller Zustand - Gelenkpositionen in Grad ")
for arg in actual_q:
    print(arg *180.0/3.1415927)

time.sleep(1)
# Roboter bewegen
input("Roboter startet Bewegung nach Eingabe beliebiger Taste")
print("🤖 moveJ..")
rtde_c.moveL([-0.600, -0.900 , 0.5,  0.0, 0.8, 0.1], 0.5, 0.3)
gripper.close()
time.sleep(1)
#gripper.move(10)  # mm

# Abschluss
rtde_c.disconnect()
rtde_r.disconnect()
