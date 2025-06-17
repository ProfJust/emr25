# Wenn das Modul außerhalb des aktuellen Projektverzeichnisses liegt, musst du Python mitteilen, dass es auch in anderen Verzeichnissen nach Modulen suchen soll. 
# Hier wird das gewünschte Verzeichnis zur Suchliste (sys.path) hinzugefügt, sodass Python das Modul finden und importieren kann
# Setze ein kleines r vor den String, damit Python die Backslashes nicht als Escape-Zeichen interpretiert
import sys
import os

# fixe Version des Pfades, bei jedem PC anders
# path2rtde_files = r"C:\mySciebo\_EMR25\emr25\ur_rtde_webot_control_lib" 

# Besser vom aktuellen Arbeitsverzeichnis ausgehen
path2rtde_files = os.getcwd() + r"\ur_rtde_webot_control_lib"
print("Add to sys ", path2rtde_files)
sys.path.insert(1, path2rtde_files)

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
# [x, y, z, roll, pitch, yaw] in rad???
waypoints_xyz = [
    [-0.45690, -0.19425, 0.06655, 0.0, 0.0, 0.0],   
    [-0.45690, -0.19425, 0.06655, 3.1, 3.1, 3.1],     
]
# Bewegungen mit MoveJ ausführen
input("Roboter startet Bewegung (MoveL) nach Eingabe beliebiger Taste")
for i, ziel_pose in enumerate(waypoints_xyz ):
    print(f"➡️  Sende moveL #{i+1}: {ziel_pose}")
    #MoveL 
    resp = rtde_c.moveL(ziel_pose)
    
    
    print(f"📥 Antwort: {resp}")
    actual_q = rtde_r.getActualQ() # in radian
    print("Aktueller Zustand - Gelenkpositionen in Grad ")
    for arg in actual_q:
        print(arg *180.0/3.1415927)
    time.sleep(2)

# Abschluss
rtde_c.disconnect()
rtde_r.disconnect()