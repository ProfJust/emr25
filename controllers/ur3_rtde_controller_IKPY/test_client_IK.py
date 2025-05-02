# test_client_IK.py
#----------------------------
# Testprogramm im RTDE-Style um den UR3e in Webots 
# für den ur3_rtde_controller
#------------------------------------------
# https://gist.github.com/ItsMichal/4a8fcb330d04f2ccba582286344dd9a7
#-------------------------------------------
# last edited by OJU 29.04.2025
#-------------------------------------------
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from robotiq_gripper_control import RobotiqGripper
import time
# import matplotlib.pyplot as plt  # pip install matplotlib
# from ikpy.utils.plot import plot_chain
# from ikpy.chain import Chain  # pip install ikpy


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
"""# input("Roboter startet Bewegung (MoveJ) nach Eingabe beliebiger Taste")
for i, ziel_pose in enumerate(waypoints_joint_angles ):
    print(f"➡️  Sende moveL #{i+1}: {ziel_pose}")
    # MoveJ
    #----------
    resp = rtde_c.moveJ(ziel_pose)
    print(f"📥 Antwort: {resp}")
    actual_q = rtde_r.getActualQ() # in radian
    print("Aktueller Zustand - Gelenkpositionen in Grad ")
    for arg in actual_q:
        print(arg *180.0/3.1415927)
    time.sleep(2)
"""
# Liste von kartesischen Zielposen 
# x,y,z, rx,ry,rz
waypoints_kartesian = [
   [ 0.5, 0.0, 0.2, 0.0, 1.0, 0.0],
   [ 0.5, 0.5, 0.2, 0.0, 1.0, 0.0],    
         
]
""" IKPy erwartet bei orientation_mode="X" einen Richtungsvektor für die X-Achse,
    nicht Euler-Winkel:
    # Falsch (Euler-Winkel):   target_orientation = [0.0, 0.0, 0.0]
    # Richtig (X-Achse zeigt in +X-Richtung):
    target_orientation = [1.0, 0.0, 0.0]  # X-Vektor (Länge > 0!)"""

# Bewegungen mit MoveL ausführen
# input("Roboter startet Bewegung (MoveL) nach Eingabe beliebiger Taste")
for i, ziel_pose in enumerate(waypoints_kartesian):
    input("\n Roboter startet Bewegung (MoveL) nach Eingabe beliebiger Taste")
    print(f"➡️  Sende moveL: {ziel_pose}")
    # MoveL 
    #------------
    # MoveL bislang ohne Inverse Kinematik, identische Funktion wie MoveJ
    # (Dummy → direkt als joint angles interpretiert)
    resp = rtde_c.moveL(ziel_pose, speed=0.5, acceleration=0.3)
    print(f"📥 Antwort: {resp}")
    time.sleep(2)    
    # Ausgabe der aktuellen Werte
    actual_q = rtde_r.getActualQ() # in radian
    print(f"➡️  Aktueller Zustand - Gelenkpositionen in RAD     : {actual_q}")  

    """# Figure initialisieren
    fig, ax = plt.subplots(figsize=(10, 10))
    ur3_chain = Chain.from_urdf_file("UR3e.urdf")
    # Kinematikkette plotten
    plot_chain(
      ur3_chain, 
      ik_solution, 
      ax=ax, 
      target=[0.5, 0.0, 0.3]  # Optional: Zielposition anzeigen
    )
    plt.show()"""

"""# Gripper Aktionen
input("Gripper startet Bewegung nach Eingabe beliebiger Taste")
gripper.close()
time.sleep(1)
input("Gripper startet Bewegung nach Eingabe beliebiger Taste")
gripper.open()
time.sleep(1)
"""

# Abschluss
rtde_c.disconnect()
rtde_r.disconnect()

