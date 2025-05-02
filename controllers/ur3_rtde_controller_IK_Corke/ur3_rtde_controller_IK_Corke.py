from controller import Robot
import socket
import threading
import json
import time

# Robotics Toolbox for Python imports für Inverse Kinematik
# python -m pip install "numpy<2"
# python -m pip install spatialmath-python roboticstoolbox-python 
import spatialmath as sm  
from roboticstoolbox import models 

# UR3-Modell für IK laden
ur3_model = models.URDF.UR3()
print(ur3_model)
# Liste aller möglichen End-Effektor-Links ausgeben:
print("EE-Links verfügbar:", ur3_model.ee_links)
# hier denjenigen wählen, an dem Dein Tool sitzt, z. B. der zweite Eintrag:
ur3_model.ee_link = ur3_model.ee_links[1]


# RTDE-Schnittstellenparameter
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 30010

# UR3e Home-Position (Gelenkwinkel in Radianten)
HOME_POSITION = [
    -0.67, -2.06, -0.62, -1.93, 0.99, 2.99
]

# Webots Roboter initialisieren
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Debug: Alle verfügbaren Devices listen
def list_devices():
    print("🔍 Verfügbare Devices:")
    for i in range(robot.getNumberOfDevices()):
        device = robot.getDeviceByIndex(i)
        print("-", device.getName())

list_devices()

# Greifer initialisieren
gripper = robot.getDevice("ROBOTIQ 2F-85 Gripper::left finger joint")
gripper_sensor = robot.getDevice("ROBOTIQ 2F-85 Gripper left finger joint sensor")
gripper_sensor.enable(timestep)

# Gelenknamen
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Motor- und Sensor-Handles
target_joint_angles = HOME_POSITION.copy()
current_joint_angles = HOME_POSITION.copy()
motors = {}
sensors = {}
for idx, name in enumerate(joint_names):
    motor = robot.getDevice(name)
    sensor = robot.getDevice(name + "_sensor")
    if motor:
        motors[name] = motor
        motor.setPosition(HOME_POSITION[idx])
    if sensor:
        sensors[name] = sensor
        sensor.enable(timestep)

lock = threading.Lock()

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
    TCP_ziel = sm.SE3(x, y, z) * sm.SE3.RPY(rx, ry, rz, order='xyz')

    # Greifer zeigt immer nach unten?
    # TCP_ziel = sm.SE3.Trans(x, y, z) * sm.SE3.OA([0, 1, 0], [0, 0, -1])
    # Trans() definiert die Position (x,y,z)
    # OA()  legt die Orientierung über Orientierungs- und Annäherungsvektor fest
    # Orientierungsvektor (Orientation Vector):
    # Dieser Vektor beschreibt eine zweite Richtung, meist die y-Achse des Endeffektor-Koordinatensystems. Er legt fest, wie der Endeffektor um den Annäherungsvektor herum gedreht ist.
    # Annäherungsvektor (Approach Vector)
    # Dieser Vektor gibt die Richtung an, in die sich der Endeffektor annähert oder „zeigt“. In der Praxis ist das oft die z-Achse des Endeffektor-Koordinatensystems.•	Dieser Vektor gibt die Richtung an, in die sich der Endeffektor annähert oder „zeigt“. In der Praxis ist das oft die z-Achse des Endeffektor-Koordinatensystems.
     
  
    # Die Methode ikine_LM() berechnet die inverse Kinematik (IK) mithilfe des Levenberg-Marquadt-Algorithmus
    #   Parameter	Beschreibung
    #   TCP_ziel	SE3-Objekt mit Zielposition und -orientierung des Endeffektors
    #   q0	        (Optional) Startschätzung für Gelenkwinkel (Standard: Aktuelle Gelenkwinkel)
    #   ilimit	    Maximale Iterationen (Standard: 30)
    #   tol	        Toleranz für Konvergenz (Standard: 1e-6)
    #   mask	    (Optional) Maskierung bestimmter Freiheitsgrade (z.B. [1,1,1,1] für nur Position)
    sol = ur3_model.ikine_LM(TCP_ziel)  # Make an IK solver
    if not sol.success:
        print("IK-Konvergenz fehlgeschlagen für Pose:", cartesian_pose)
        return None
    return sol.q.tolist()

def handle_client(conn, addr):
    print(f"[Server] Neue Verbindung von {addr}")
    global current_joint_angles, target_joint_angles
    try:
        while True:
            raw = conn.recv(4096)
            print("[Server] Roh-Request:", raw)
            if not raw:
                break
            try:
                request = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                conn.sendall(json.dumps({"error": "Ungültiges JSON"}).encode("utf-8"))
                continue

            command = request.get("command")
            response = {}

            with lock:
                if command == "getActualQ":
                    response = {"data": current_joint_angles.copy()}

                elif command == "moveJ":
                    joint_targets = request.get("data")
                    speed = request.get("speed", 0.5)
                    if isinstance(joint_targets, list) and len(joint_targets) == 6:
                        target_joint_angles[:] = joint_targets
                        for name in joint_names:
                            motors[name].setVelocity(speed)
                        response = {"info": "moveJ akzeptiert", "target_q": joint_targets, "speed": speed}

                elif command == "moveL":
                    payload = request.get("data", {})
                    pose = payload.get("pose", [])
                    speed = payload.get("speed", 0.5)
                    if len(pose) == 6:
                        q = inverse_kinematics(pose)
                        if q:
                            target_joint_angles[:] = q
                            for name in joint_names:
                                motors[name].setVelocity(speed)
                            response = {"info": "moveL akzeptiert", "target_q": q, "speed": speed}
                        else:
                            response = {"error": "IK fehlgeschlagen"}
                    else:
                        response = {"error": "Ungültige Pose"}

                elif command == "reset_to_home":
                    target_joint_angles[:] = HOME_POSITION.copy()
                    response = {"info": "Zurück zur Home-Position"}

                elif command == "openGripper":
                    gripper.setPosition(0.01)
                    sensor_value = gripper_sensor.getValue()
                    response = {"info": "Greifer geöffnet", "sensor": sensor_value}

                elif command == "closeGripper":
                    gripper.setPosition(0.8)
                    sensor_value = gripper_sensor.getValue()
                    response = {"info": "Greifer geschlossen", "sensor": sensor_value}

                elif command == "disconnect":
                    response = {"info": "Verbindung getrennt"}
                    conn.sendall(json.dumps(response).encode("utf-8"))
                    conn.shutdown(socket.SHUT_WR)
                    time.sleep(0.05)
                    break

                else:
                    response = {"error": "Unbekannter Befehl"}

            print("[Server] Antwort-Payload:", response)
            conn.sendall(json.dumps(response).encode("utf-8"))
    except Exception as e:
        print(f"❌ Fehler: {e}")
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

# Server-Thread starten
threading.Thread(target=server_thread, daemon=True).start()

# Hauptsteuerung
while robot.step(timestep) != -1:
    with lock:
        current_joint_angles = [
            sensors[name].getValue() if name in sensors else 0.0
            for name in joint_names
        ]
        for name, angle in zip(joint_names, target_joint_angles):
            if name in motors:
                motors[name].setPosition(angle)

