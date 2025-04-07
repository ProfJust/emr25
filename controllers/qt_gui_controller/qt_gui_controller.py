# qt_gui_controller.py
# ----------------------------------------------------------------------
# EMR25 
# OJ 7.4.25
#
# Skript für den Aufgabu eines Qt-GUI zum steuern des UR3e-Roboters 
# in Webots
# 
# Achtung: Dieses Skript zuerst starten, dann webots 
# Ansonsten gibt es einen Socket_Error
#-----------------------------------------------------------------------


from controller import Robot
import socket
import threading

# Zeitstufe setzen
timeStep = 32

# Roboter initialisieren
robot = Robot()

# Motoren initialisieren
motor1 = robot.getDevice('shoulder_lift_joint')
motor2 = robot.getDevice('elbow_joint')
motor3 = robot.getDevice('wrist_1_joint')
motor4 = robot.getDevice('wrist_2_joint')

# Nachrichten-Warteschlange
message_queue = []

def handle_socket():
    global message_queue
    # TCP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30) 
    sock.bind(('localhost', 12345))
    sock.listen(1)
    
    while True:
        conn, addr = sock.accept()
        try:
            data = conn.recv(1024)
        except socket.timeout:
            continue
        message_queue.append(data)
        conn.close()

# Socket-Thread starten
socket_thread = threading.Thread(target=handle_socket)
socket_thread.daemon = True  # Damit der Thread beim Beenden des Hauptprogramms beendet wird
socket_thread.start()

while robot.step(timeStep) != -1:
    if message_queue:
        data = message_queue.pop(0)
        if data == b'move_up':
            value = -0.5
        elif data == b'move_down':
            value = +0.5  # Bewegt den Arm runter
        motor1.setPosition(value)  # Bewegt den Arm
