from controller import Robot
import socket
import threading
import json
import time

# RTDE-Schnittstellenparameter
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 30010

# UR3e Home-Position
HOME_POSITION = [
    -0.67, -2.06, -0.62, -1.93, 0.99, 2.99
]


# Initialisiere Webots-Roboter
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Gelenkkonfiguration
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

# Initialisiere Motoren und Sensoren
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

# Initialisierung des Grippers
gripper = robot.getDevice("ROBOTIQ_2F85::left finger joint")
gripper_sensor = robot.getDevice("ROBOTIQ_2F85 left finger joint sensor")
gripper_sensor.enable(timestep)

# Globale Variablen
current_joint_angles = HOME_POSITION.copy()
target_joint_angles = HOME_POSITION.copy()
lock = threading.Lock()

        
def inverse_kinematics(cartesian_pose):
    return cartesian_pose if len(cartesian_pose) == 6 else None

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
                    pose = request.get("data", {}).get("pose", [])
                    if len(pose) == 6:
                        target_joint_angles[:] = inverse_kinematics(pose) or current_joint_angles
                        response = {"info": "moveL akzeptiert", "target_q": target_joint_angles.copy()}
                    else:
                        response = {"error": "Ungültige Pose"}

                elif command == "reset_to_home":
                    target_joint_angles[:] = HOME_POSITION.copy()
                    response = {"info": "Zurück zur Home-Position"}
                
                elif command == "OpenGripper":
                     gripper.setPosition(0.02)  # Fully opened
                     
                elif command == "CloseGripper":
                     gripper.setPosition(0.7)  # Fully closed
                    
                elif command == "disconnect":
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
    with lock:
        # Aktuelle Gelenkpositionen aktualisieren
        current_joint_angles = [
            sensors[name].getValue() if name in sensors else 0.0
            for name in joint_names
        ]
        
        # Roboterbewegung steuern
        for name, angle in zip(joint_names, target_joint_angles):
            if name in motors:
                motors[name].setPosition(angle)
