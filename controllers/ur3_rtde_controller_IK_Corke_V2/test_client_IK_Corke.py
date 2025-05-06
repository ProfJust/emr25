from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from robotiq_gripper_control import RobotiqGripper

import sys
print(sys.executable)
# C:\Users\olafj\AppData\Local\Programs\Python\Python36-32\python.exe -m pip install roboticstoolbox-python

from roboticstoolbox import models 
# pip install roboticstoolbox-python
# import spatialmath as sm
import time
import math

def get_actual_tcp():
    # Aktueller TCP in kartesichen Koordinaten ermitteln 
    _q0 = rtde_r.getActualQ()
    T_home = robot.fkine(_q0)
    # print (T_home)
    _x0, _y0, _z0 = T_home.t.flatten().tolist()
    print ("Aktuelle TCP Position x0, y0, z0", _x0, _y0, _z0)
    return _x0, _y0, _z0


UR3_IP = "127.0.0.1"
rtde_r = RTDEReceiveInterface(UR3_IP)
rtde_c = RTDEControlInterface(UR3_IP)
gripper = RobotiqGripper(rtde_c)
gripper.connect(UR3_IP)

# Forward-Kinematics-Objekt
robot = models.URDF.UR3()

# Aktueller TCP in kartesichen Koordinaten ermitteln 
x0, y0, z0 = get_actual_tcp()
input("Bestätigen, wenn Roboter zu Position 1 fahren soll")
# Pose 1: 0.10m hoch
#       [ x , y  , z  , rx,  , ry , rz ] 
pose1 = [x0 , y0-0.1  , z0 , 0.0, 0.0, 0.0]  # math.pi = 3.12...
resp = rtde_c.moveL(pose1, speed=0.3, acceleration=0.3)
input("Fahrt beendet?")
#time.sleep(10)  #wait till robot is ready
x0, y0, z0 = get_actual_tcp()
input("Bestätigen, wenn Roboter zu Position 2 fahren soll")

# Pose 2: 0,1m in Y-Richtung
#       [ x , y  , z  , rx,  , ry , rz ] 
pose2 = [x0, y0+0.1  , z0-0.1 , 0.0, 0.0, 0.0]
resp = rtde_c.moveL(pose2, speed=0.3, acceleration=0.3)

input("Fahrt beendet?")
x0, y0, z0 = get_actual_tcp()

# Verbindung trennen
rtde_c.disconnect()
rtde_r.disconnect()
