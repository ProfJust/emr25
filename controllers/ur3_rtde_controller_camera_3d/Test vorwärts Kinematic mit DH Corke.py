# Test vorwärts Kinematic mit DH
#-----------------------------------------------
# Variante mit der Corke - Toolbox
#
# sollte für die neutrale Pose ergeben
#  [-0.45690, -0.19425, 0.06655] laut UR-Excel-Sheet
# OK, tut es auch!! 
# Tested as OK!
# OJ 27.05.2025

import roboticstoolbox as rtb
import numpy as np

# DH-Parameter für UR3e (a, d, alpha, theta)
# https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

dh_params = [
    [ 0.0000, 0.1519,   np.pi/2, 0],    # Gelenk 1 Base
    [-0.2437, 0.0000,   0.00000, 0],    # Gelenk 2 Shoulder
    [-0.2133, 0.0000,   0.00000, 0],    # Gelenk 3 Elbow
    [ 0.0000, 0.11235,  np.pi/2, 0],    # Gelenk 4 Wrist1
    [ 0.0000, 0.08535, -np.pi/2, 0],    # Gelenk 5 Wrist 2
    [ 0.0000, 0.0819,         0, 0]     # Gelenk 6 Wrist 3
]

# Roboter mit DH-Parametern erstellen
ur3e = rtb.DHRobot([
    rtb.RevoluteDH(d=link[1], a=link[0], alpha=link[2]) for link in dh_params
])

# Neutrale Gelenkwinkel (θ1–θ6 = 0)
q = np.array([0, 0, 0, 0, 0, 0])

# Vorwärtskinematik berechnen
T = ur3e.fkine(q)
print("TCP-Pose (SE3-Objekt):\n", T)
print("TCP-Position (x, y, z):", T.t)
print("TCP-Orientierung (Rotationsmatrix):\n", T.R)

"""Erwartetes Ergebnis für neutrale Pose
text
TCP-Position (x, y, z): [-0.4565, 0, 0.6655]
"""

ur3e.plot(q, backend='pyplot')  # 3D-Plot anzeigen
input()


