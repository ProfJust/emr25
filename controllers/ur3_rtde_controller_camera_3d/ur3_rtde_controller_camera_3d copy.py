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
from controller import Robot
#from controller import Supervisor  
#nutze robot für alle weiteren Gerätezugriffe (Kamera, Motoren, etc.), da Supervisor von Robot erbt.
import socket
import threading
import json
import time

# RTDE-Schnittstellenparameter
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 30010

# UR3e Home-Position
HOME_POSITION = [
    1.77, -1.00, 1.76, -1.56, 2.33, 1.64
]

# Gripper-Konfiguration (Anpassen an Ihre Webots-Welt!)
GRIPPER_OPEN_POS = 0.04  # Maximal geöffnet (in Rad)
GRIPPER_CLOSE_POS = 0.0  # Geschlossen
GRIPPER_SPEED = 1.0      # Rad/s

# Initialisiere Webots-Roboter
robot = Robot() # Supervisor() ??
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

"""#### ADD TCP Positionn Supervisor ####
# Node-Name des TCP, z.B. 'UR3e_tcp' im Scene Tree
# Siehe DEF im Proto Files
node_name = 'INNER_FINGER_PAD'
tcp_node = robot.getFromDef(node_name)
if tcp_node is None:
    print("❌ Kein Node mit DEF",node_name, " gefunden!")
else:
    tcp_pose = tcp_node.getPosition()
"""

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

    """tcp_pose = tcp_node.getPosition()
    tcp_orientation = tcp_node.getOrientation()
    print(" TCP Posn:", tcp_pose, end=" ")
    print(" TCP Ortn:", tcp_orientation)
    """
   
    with lock:
        # Aktuelle Gelenkpositionen aktualisieren
        current_joint_angles = [
            sensors[name].getValue() if name in sensors else 0.0
            for name in joint_names
        ]
        print("aktuelle Gelenkwinkel ", current_joint_angles )
        # Roboterbewegung steuern
        for name, angle in zip(joint_names, target_joint_angles):
            if name in motors:
                motors[name].setPosition(angle)
        
       