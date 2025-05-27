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

# RTDE-Schnittstellenparameter
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 30010

# UR3e Home-Position 
#HOME_POSITION =[ 1.77, 1.00, 1.76, -1.56, 2.33, 1.64] #irgendeine


# HOME_POSITION = [3.894989504502283e-06, -1.1196710709038271e-05, 3.75, 0.00021631375190624204, -7.870102380896915e-05, -4.415480779567577e-06]
# Neutrale Pose  0.0, 0.0, 0.0, 0.0, 0.0, 0.0 # 

HOME_POSITION = [ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]  # in rad
#HOME_POSITION_GRAD = [ 0.00, -87.73, -7.63, -0.28, 0.87, 0.22]  #Foto aufrechte Position
HOME_POSITION_GRAD = [ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
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
tool_slot = robot.getFromDef("TOOL_SLOT")  

timestep = int(robot.getBasicTimeStep())

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

#### TCP ermitteln ##############################################################
import roboticstoolbox as rtb
import numpy as np

# DH-Parameter für UR3e (a, d, alpha, theta)
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

# Neutrale Gelenkwinkel (θ1–θ6 = 0)
# q = np.array([0, 0, 0, 0, 0, 0])

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
    # Pose x,y,z, rx,ry,rz  => 6 Winkel 
    angles = cartesian_pose # Dummy-Implementierung
    return angles if len(angles) == 6 else None
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
                    # hier die IK einfügen
                    pose = request.get("data", {}).get("pose", [])
                    #  pose wird im Dummy als Winkel interpretiert
                    if len(pose) == 6:
                        # IK muss noch in Funktion inverse_kinematics() realisiert werden
                        target_joint_angles[:] = inverse_kinematics(pose) or current_joint_angles
                        response = {"info": "moveL akzeptiert", "target_q": target_joint_angles.copy()}
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
        # print(f"aktuelle Gelenkwinkel: {current_joint_angles[0]:.2f}, {current_joint_angles[1]:.2f}, {current_joint_angles[2]:.2f}, {current_joint_angles[3]:.2f}, {current_joint_angles[4]:.2f}, {current_joint_angles[5]:.2f}",     end=" ")
        #print("aktuelle Gelenkwinkel: ", current_joint_angles, end=" ")                               
          
        if tool_slot:
            print("Tool-Slot-Position:", tool_slot.getPosition())
        else:
            print("DEF 'TOOL_SLOT' nicht gefunden. Proto-Datei prüfen!")                 
        #####  TCP ##################           
        # Vorwärtskinematik berechnen
        T = ur3e.fkine(current_joint_angles)  # fkine-Methode ergibt ein SE3-Objekt
        print(f"TCP-Position (x, y, z): {T.t[0]:.2f}, {T.t[1]:.2f}, {T.t[2]:.2f}")    #, end=" "
        # TESTED 27.05.25: Ausgabe Entspricht der EXCEL-Tabelle von UR
        #
        # Leider ist der Robotiq_Gripper in Webots an einer anderen Position
        # Bsp: Ausgabe hier
        # aktuelle Gelenkwinkel: 0.00, -0.00, 3.75, 0.00, -0.00, -0.00 TCP-Position (x, y, z): -0.12, -0.37, 0.34
        # Robotiq Pos realtiv zu UR3e                                                          -0.138 0.163  0.623
        # Robotiq Pos absolut                                                                  -0.163 -0.138  1.563



        # Wichtig? 
        # Webots verwendet ein Y-up-System, während UR-Roboter typischerweise Z-up-Konfigurationen nutzen.
        #  Dies führt zu unterschiedlichen Rotationsmatrizen in der Vorwärtskinematik    
        # print(f" real (x, y, z): {T.t[0]:.2f}, {T.t[2]:.2f}, {T.t[1]:.2f}")

        # Roboterbewegung steuern
        for name, angle in zip(joint_names, target_joint_angles):
            if name in motors:
                motors[name].setPosition(angle)
        
       