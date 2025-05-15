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

# Liste von Positionen (Gelenkwinkel)
waypoints_joint_angles = [
    [-0.19, -2.10, -1.67, -2.75, 0.99, 2.99],   
    [-0.62, -1.38, -1.67, -1.81, 0.37, 2.99],     
]
# Bewegungen mit MoveJ ausführen
input("Roboter startet Bewegung (MoveJ) nach Eingabe beliebiger Taste")
for i, ziel_pose in enumerate(waypoints_joint_angles ):
    print(f"➡️  Sende moveL #{i+1}: {ziel_pose}")
    #MoveL resp = rtde_c.moveL(ziel_pose, speed=0.5, acceleration=0.3)
    resp = rtde_c.moveJ(ziel_pose)
    print(f"📥 Antwort: {resp}")
    actual_q = rtde_r.getActualQ() # in radian
    print("Aktueller Zustand - Gelenkpositionen in Grad ")
    for arg in actual_q:
        print(arg *180.0/3.1415927)
    time.sleep(2)

# Gripper Aktionen
input("Gripper startet Bewegung nach Eingabe beliebiger Taste")
gripper.close()
time.sleep(1)
input("Gripper startet Bewegung nach Eingabe beliebiger Taste")
gripper.open()
time.sleep(1)
gripper.close()
time.sleep(1)

# Abschluss
rtde_c.disconnect()
rtde_r.disconnect()