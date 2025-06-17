# Test inverse Kinematic mit DH
#-----------------------------------------------
# Variante mit der Corke - Toolbox
#
# sollte für [-0.45690, -0.19425, 0.06655]
# die neutrale Pose ergeben
#   laut UR-Excel-Sheet

import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

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


# Zielpose definieren (Beispiel)
T_goal = SE3.Trans(-0.45690, -0.19425, 0.06655) * SE3.Rx(180, 'deg')
print(T_goal)

# Schnelle IK mit Levenberg-Marquad
sol = ur3e.ikine_LM(T_goal)
if sol.success:
    print(f"Gelenkwinkel: {sol.q}")
else:
    print("Keine Lösung gefunden")

#Umgang mit Mehrfachlösungen
for seed in np.linspace(-np.pi, np.pi, 10):
    sol = ur3e.ikine_LM(T_goal, q0=seed)
    if sol.success:
        print(f"Alternative Lösung: {sol.q}")
    else:
        print("Keine alternative Lösung gefunden")


ur3e.plot(sol.q, backend='pyplot')  # 3D-Plot anzeigen
input()


