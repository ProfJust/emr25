# ur3_rtde_controller_camera.py
#-------------------------------------------------
# Westfälische Hochschule - Campus Bocholt
# Michael Engelmann und Olaf Just
#-------------------------------------------------
# Ergänzt die Camera im Controller in der 
# emr25_world_3_Tisch_Camera.wbt 
# -------------------------------------------------
# 
# OJU 19.05.2025
#----------------------------
from controller import Supervisor # was Robot
import socket
import threading
import json
import time
import math as m
import os


# RTDE-Schnittstellenparameter
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 30010

# UR3e Home-Position 
#HOME_POSITION =[ 1.77, 1.00, 1.76, -1.56, 2.33, 1.64] #irgendeine


# HOME_POSITION = [3.894989504502283e-06, -1.1196710709038271e-05, 3.75, 0.00021631375190624204, -7.870102380896915e-05, -4.415480779567577e-06]
# Neutrale Pose  0.0, 0.0, 0.0, 0.0, 0.0, 0.0 # 

HOME_POSITION = [ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]  # in rad
#HOME_POSITION_GRAD = [ 0.00, -87.73, -7.63, -0.28, 0.87, 0.22]  #Foto aufrechte Position
HOME_POSITION_GRAD = [ 0.00, 0.00, 180.00, 0.00, 0.00, 0.00]
i = 0
for wert in HOME_POSITION_GRAD: 
    HOME_POSITION[i] = wert *m.pi/190 # m.radians(wert)
    i = i+1
print ("Home Pos: ", HOME_POSITION)

# Gripper-Konfiguration (Anpassen an Ihre Webots-Welt!)
GRIPPER_OPEN_POS = 0.04  # Maximal geöffnet (in Rad)
GRIPPER_CLOSE_POS = 0.0  # Geschlossen
GRIPPER_SPEED = 1.0      # Rad/s

# Initialisiere Webots-Roboter
robot = Supervisor() # was Robot() 

#https://github.com/cyberbotics/webots/blob/released/projects/samples/devices/controllers/gps_supervisor/gps_supervisor.py
#tool_slot = robot.getFromDef("TOOL_SLOT").getField('translation')  #getFromDef('GPS_ROBOT').getField('translation')

timestep = int(robot.getBasicTimeStep())  # Wie oft pro Sekunde (aktuell 32 mal)

##### ADD CAMERA ###################
camera = robot.getDevice('cam1')  # 'cam' must match Camera node name
camera.enable(timestep)          # timestep must be >0
print("cam", camera)
# s.u. => while robot.step(timestep) != -1:
    #image = camera.getImage()
    # Bildverarbeitung hier
    
##### ADD 3D_CAMERA ###################    
range_finder = robot.getDevice("my_range_finder")
range_finder.enable(timestep)

###### ADD GPS @ TCP ########
#gps = robot.getDevice("tool_slot_gps")
#gps.enable(timestep)

#### TCP ermitteln ##############################################################
import roboticstoolbox as rtb
import numpy as np

# DH-Parameter genau wie für den realen UR3e (a, d, alpha, theta)
dh_params = [
    [ 0.0000, 0.1519,   np.pi/2, 0],    # Gelenk 1 Base
    [-0.2437, 0.0000,   0.00000, 0],    # Gelenk 2 Shoulder
    [-0.2133, 0.0000,   0.00000, 0],    # Gelenk 3 Elbow
    [ 0.0000, 0.11235,  np.pi/2, 0],    # Gelenk 4 Wrist1
    [ 0.0000, 0.08535, -np.pi/2, 0],    # Gelenk 5 Wrist 2
    [ 0.0000, 0.2559,         0, 0]     # Gelenk 6 Wrist 3    d= 0.0819 mit Gripper + 0.174 => 0.2559
]
# Roboter mit DH-Parametern erstellen
ur3e = rtb.DHRobot([
    rtb.RevoluteDH(d=link[1], a=link[0], alpha=link[2]) for link in dh_params
])
#### TCP ermitteln ##############################################################

# Debug: Zeige alle verfügbaren Devices
print("🔍 Verfügbare Devices:")
for i in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(i)
    print("-", device.getName())
    
# Gripper initialisieren
gripper = robot.getDevice("ROBOTIQ 2F-85 Gripper::left finger joint")
gripper_sensor = robot.getDevice("ROBOTIQ 2F-85 Gripper left finger joint sensor")
gripper_sensor.enable(timestep)

# Gelenkkonfiguration
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Initialisiere Roboter-Motoren und Sensoren
motors = {}
sensors = {}
for name in joint_names:
    motor = robot.getDevice(name)
    sensor = robot.getDevice(name + "_sensor")
    if motor:
        motors[name] = motor
        motor.setPosition(HOME_POSITION[joint_names.index(name)])
    if sensor:
        sensor.enable(timestep)
        sensors[name] = sensor

# Globale Variablen
current_joint_angles = HOME_POSITION.copy()
target_joint_angles = HOME_POSITION.copy()
lock = threading.Lock()

#---------- HIER DIE IK UMSETZEN ----!!!
def inverse_kinematics(cartesian_pose):
    """
    Berechnet IK für eine gegebene kartesische Pose.
    cartesian_pose: [x, y, z, roll, pitch, yaw]
    Gibt eine Liste von 6 Gelenkwinkeln (rad) oder None zurück.
    """
    if len(cartesian_pose) != 6:
        print("Ungültige Pose für IK:", cartesian_pose)
        return None
    x, y, z, rx, ry, rz = cartesian_pose   # roll rx, pitch ry, yaw rz

    # Matrix für die Zielpose (Endeffektor-Pose) definieren
    # sm = spatialmath Bibliothek   
    # Position (x,y,z) und Orientierung (rx,ry,rz)
    Tep = sm.SE3(x, y, z) * sm.SE3.RPY(rx, ry, rz, order='xyz')
    # print(" TCP Ziel: ", TCP_ziel)
    # Zielpose definieren (Beispiel: 10 cm vorwärts in x-Richtung)
    #TCP_ziel = sm.SE3.Trans(0.1, 0, 0) * sm.SE3.RPY([0, 0, 0])

    # Greifer zeigt immer nach unten?
    # TCP_ziel = sm.SE3.Trans(x, y, z) * sm.SE3.OA([0, 1, 0], [0, 0, -1])
    # Trans() definiert die Position (x,y,z)
    # OA()  legt die Orientierung über Orientierungs- und Annäherungsvektor fest
    # Orientierungsvektor (Orientation Vector):
    # Dieser Vektor beschreibt eine zweite Richtung, meist die y-Achse des Endeffektor-Koordinatensystems. Er legt fest, wie der Endeffektor um den Annäherungsvektor herum gedreht ist.
    # Annäherungsvektor (Approach Vector)
    # Dieser Vektor gibt die Richtung an, in die sich der Endeffektor annähert oder „zeigt“. In der Praxis ist das oft die z-Achse des Endeffektor-Koordinatensystems.•	Dieser Vektor gibt die Richtung an, in die sich der Endeffektor annähert oder „zeigt“. In der Praxis ist das oft die z-Achse des Endeffektor-Koordinatensystems.
     
  
    # Die Methode ikine_LM() berechnet die inverse Kinematik (IK) mithilfe des Levenberg-Marquadt-Algorithmus
    # https://petercorke.github.io/robotics-toolbox-python/IK/stubs/roboticstoolbox.robot.Robot.Robot.ikine_LM.html
    #   Parameter	Beschreibung
    #   TCP_ziel	SE3-Objekt mit Zielposition und -orientierung des Endeffektors
    #   q0	        (Optional) Startschätzung für Gelenkwinkel (Standard: Aktuelle Gelenkwinkel)
    #   ilimit	    Maximale Iterationen (Standard: 30)
    #   tol	        Toleranz für Konvergenz (Standard: 1e-6)
    #   mask	    (Optional) Maskierung bestimmter Freiheitsgrade (z.B. [1,1,1,1] für nur Position)

    end =  ur3_model.ee_links[2] # tool_0
    print(end)
    #print(ur3_model)
    start = ur3_model.links[1] # base_links
    print(start)
    #q0 = ur3_model.q  ############## The initial joint coordinate vector is Zero  => ERROR ???
    q0 = current_joint_angles.copy()
    print("Aktuelle Gelenkwinkel", q0)

    sol = ur3_model.ikine_LM(
        Tep,
        end,
        start, 
        q0
    )  # Make an IK solver
    
    
    # Analytische Lösung => ERROR 'UR3' object has no attribute 'ikine_a'
    # sol = ur3_model.ikine_a(TCP_ziel, config="lun")  # Make an IK solver

    if not sol.success:
        print("IK-Konvergenz fehlgeschlagen für Pose:", cartesian_pose)
        return None
    return sol.q.tolist()
#---------- HIER DIE IK UMSETZEN ----!!!


def set_gripper(position):
    if gripper_motor:
        gripper_motor.setPosition(position)
        time.sleep(0.5)  # Für die Simulation

def handle_client(conn, addr):
    print(f"🔗 Neue Verbindung von {addr}")
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            
            try:
                request = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                response = {"error": "Ungültiges JSON"}
                conn.sendall(json.dumps(response).encode("utf-8"))
                continue

            command = request.get("command")
            response = {}

            with lock:
                if command == "getActualQ":
                    response = {"data": current_joint_angles.copy()}

                elif command == "moveJ":
                    joint_targets = request.get("data")
                    if isinstance(joint_targets, list) and len(joint_targets) == 6:
                        target_joint_angles[:] = joint_targets
                        response = {"info": "moveJ akzeptiert", "target_q": joint_targets}
                    else:
                        response = {"error": "Ungültige Gelenkwinkel"}

                elif command == "moveL":
                    # hier den IK-Aufruf einfügen
                    payload = request.get("data", {})
                    pose = payload.get("pose", [])
                    speed = payload.get("speed", 0.5)
                    if len(pose) == 6:
                        print(" Target Pose IK ", pose)
                        q = inverse_kinematics(pose)  # ==>>> IK 
                        if q:
                            target_joint_angles[:] = q
                            for name in joint_names:
                                motors[name].setVelocity(speed)
                            response = {"info": "Target Q akzeptiert", "target_q": q, "speed": speed}
                        else:
                            response = {"error": "IK fehlgeschlagen"}
                    else:
                        response = {"error": "Ungültige Pose"}

                elif command == "reset_to_home":
                    target_joint_angles[:] = HOME_POSITION.copy()
                    response = {"info": "Zurück zur Home-Position"}

                # Neue Gripper-Befehle
                elif command == "openGripper":
                    gripper.setPosition(0.01)  # Fully opened
                    sensor_value = gripper_sensor.getValue()
                    print("Griffweite:",sensor_value)
                    response = {"info": "Greifer geöffnet"}

                elif command == "closeGripper":
                    gripper.setPosition(0.8)  # Fully close 
                    sensor_value = gripper_sensor.getValue()
                    print("Griffweite:",sensor_value)
                    response = {"info": "Greifer geschlossen"}

                elif command == "disconnect":
                    response = {"info": "Verbindung getrennt"}
                    conn.sendall(json.dumps(response).encode("utf-8"))
                    conn.shutdown(socket.SHUT_WR)
                    time.sleep(0.05)  # kurze Pause, damit der Client noch lesen kann
                    break

                else:
                    response = {"error": "Unbekannter Befehl"}

            conn.sendall(json.dumps(response).encode("utf-8"))

    except Exception as e:
        print(f"❌ Fehler: {str(e)}")
    finally:
        conn.close()

def server_thread():
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind((SERVER_HOST, SERVER_PORT))
    server_sock.listen(1)
    print(f"🚀 Server aktiv auf {SERVER_HOST}:{SERVER_PORT}")
    
    while True:
        conn, addr = server_sock.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

# Starte Server-Thread
threading.Thread(target=server_thread, daemon=True).start()

#GUI_start = True
tstep = 1
# Starte GUI als separaten Prozess
#import subprocess
#subprocess.Popen(['python', 'c:/mySciebo/_EMR25/emr25/Py_Qt6/Webots_GUI_Joint_Angle_Control.py'])

#from PyQt5.QtCore import QProcess
#process = QProcess()
#process.start('python', 'c:/mySciebo/_EMR25/emr25/Py_Qt6/Webots_GUI_Joint_Angle_Control.py')

# Hauptsteuerungsschleife
while robot.step(timestep) != -1:
    # get image
    image = camera.getImage()
    # save image to file, ACHTUNG: hält die Schleife auf!!!
    #camera.saveImage("image1.jpg", 100)  # 100 = quality (for JPEG)
    # while robot.step(timestep) != -1:
    #image = camera.getImage()
    # Bildverarbeitung hier
    
    rf_image = range_finder.getRangeImage()
    width = range_finder.getWidth()
    height = range_finder.getHeight()
    y = 160 # Koordinaten Bildpunkt
    x = 120
    distance = rf_image[y * width + x] # Zugriff auf einen einzelnen Wert
    #print("Distance ", distance)
    #range_finder.saveImage("image_rf1.jpg", 100)
   
    with lock:
        # Aktuelle Gelenkpositionen aktualisieren
        current_joint_angles = [
            sensors[name].getValue() if name in sensors else 0.0
            for name in joint_names
        ]
        #formatierte Ausgabe
        print(f"aktuelle Gelenkwinkel: {tstep}, {current_joint_angles[0]:+.2f}, {current_joint_angles[1]:+.2f}, {current_joint_angles[2]:+.2f}, {current_joint_angles[3]:+.2f}, {current_joint_angles[4]:+.2f}, {current_joint_angles[5]:+.2f}",     end=" ")
        tstep = tstep +1
        """ NO WORX  
        if tool_slot:
            print("Tool-Slot-Position:", tool_slot.getPosition())
        else:
            print("DEF 'TOOL_SLOT' nicht gefunden. Proto-Datei prüfen!")                 
        """
        #####  TCP ##################           
        # Vorwärtskinematik berechnen
        T = ur3e.fkine(current_joint_angles)  # fkine-Methode ergibt ein SE3-Objekt
        print(f" TCP-Position (x, y, z): {T.t[0]:+.2f}, {T.t[1]:+.2f}, {T.t[2]:+.2f}") # , end=" ")
        
        ###### GPS ###########
        #gps_position = gps.getValues()
        #print(f"  GPS {gps_position[0]:.2f}, {gps_position[1]:.2f}, {gps_position[2]:.2f}")

        # Roboterbewegung steuern
        for name, angle in zip(joint_names, target_joint_angles):
            if name in motors:
                motors[name].setPosition(angle)

        """ if GUI_start:
            GUI_start = False
            # Starte GUI als separaten Prozess
            import subprocess
            subprocess.Popen(['python', 'c:/mySciebo/_EMR25/emr25/Py_Qt6/Webots_GUI_Joint_Angle_Control.py'])
        """
       