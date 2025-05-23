# Test vorwärts Kinematic mit DH
# sollte ergeben [-0.4565, 0, 0.6655]
import roboticstoolbox as rtb
import numpy as np

# DH-Parameter für UR3e (a, d, alpha, theta)
dh_params = [
    [0, 0.1519, np.pi/2, 0],    # Gelenk 1
    [0.2435, 0, 0, 0],          # Gelenk 2
    [0.213, 0, 0, 0],           # Gelenk 3
    [0, 0.11235, np.pi/2, 0],   # Gelenk 4
    [0, 0.08535, -np.pi/2, 0],  # Gelenk 5
    [0, 0.0921, 0, 0]           # Gelenk 6
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


