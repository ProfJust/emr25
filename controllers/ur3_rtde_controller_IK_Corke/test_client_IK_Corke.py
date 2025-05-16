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


print(sys.executable)
# C:\Users\olafj\AppData\Local\Programs\Python\Python36-32\python.exe -m pip install roboticstoolbox-python

from roboticstoolbox import models 
# pip install roboticstoolbox-python
# import spatialmath as sm
import time
import math

UR3_IP = "127.0.0.1"
rtde_r = RTDEReceiveInterface(UR3_IP)
rtde_c = RTDEControlInterface(UR3_IP)
gripper = RobotiqGripper(rtde_c)
gripper.connect(UR3_IP)

# Home-Gelenkwinkel
HOME_Q = [-0.67, -2.06, -0.62, -1.93, 0.99, 2.99]

# Forward-Kinematics-Objekt
robot = models.UR3()

# 1) FK(Home) ermitteln und Pose 1 definieren
T_home = robot.fkine(HOME_Q)
x0, y0, z0 = T_home.t.flatten().tolist()

# Pose 1: 0.30m vor (Y-Richtung), +0.20m hoch, Tool nach unten zeigen
pose1 = [x0, y0 - 0.30, z0 + 0.20, math.pi, 0.0, 0.0]
print("➡️ MoveL zu Pose 1 (vor & hoch):", pose1)
resp = rtde_c.moveL(pose1, speed=0.3, acceleration=0.3)
print("Antwort:", resp)
input("Bestätigen, wenn Position 1 erreicht ist…")

# 2) Senkrecht 0.15m nach unten absenken
pose2 = [pose1[0], pose1[1], pose1[2] - 0.15, pose1[3], pose1[4], pose1[5]]
print("➡️ MoveL zu Pose 2 (senk. runter):", pose2)
resp = rtde_c.moveL(pose2, speed=0.3, acceleration=0.2)
print("Antwort:", resp)
input("Bestätigen, wenn Position 2 erreicht ist…")

# Greifer schließen
print("🔧 Greifer schließen")
gripper.move_and_wait_for_pos(0, 255, 255)
print("Greifer-Status:", gripper.get_current_position())

# Verbindung trennen
rtde_c.disconnect()
rtde_r.disconnect()
