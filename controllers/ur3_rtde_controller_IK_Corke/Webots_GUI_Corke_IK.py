import sys
import json
import socket
import math
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

# Für Forward Kinematics
import spatialmath as sm
from roboticstoolbox import models

class RobotControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ur3_model = models.UR3()
        self.initUI()
        self.connect_to_robot()
        # Einmalige Initialabfrage der aktuellen Pose
        self.request_current_pose()

    def initUI(self):
        self.setWindowTitle('UR3e Cartesian Control (MoveL)')
        layout = QVBoxLayout()

        # Eingabefelder für kartesische Pose, angepasst auf UR3-Arbeitsbereich
        form = QFormLayout()
        # UR3 Arbeitsbereich: X und Y in ±0.42m, Z in [0.0, 0.8]
        self.spin_x = QDoubleSpinBox(); self.spin_x.setRange(-0.42, 0.42); self.spin_x.setSingleStep(0.01)
        self.spin_y = QDoubleSpinBox(); self.spin_y.setRange(-0.42, 0.42); self.spin_y.setSingleStep(0.01)
        self.spin_z = QDoubleSpinBox(); self.spin_z.setRange(0.0, 0.8); self.spin_z.setSingleStep(0.01)
        # Orientierung: Roll, Pitch, Yaw in [-π, π]
        self.spin_roll = QDoubleSpinBox(); self.spin_roll.setRange(-math.pi, math.pi); self.spin_roll.setSingleStep(0.1)
        self.spin_pitch = QDoubleSpinBox(); self.spin_pitch.setRange(-math.pi, math.pi); self.spin_pitch.setSingleStep(0.1)
        self.spin_yaw = QDoubleSpinBox(); self.spin_yaw.setRange(-math.pi, math.pi); self.spin_yaw.setSingleStep(0.1)

        form.addRow('X [m]:', self.spin_x)
        form.addRow('Y [m]:', self.spin_y)
        form.addRow('Z [m]:', self.spin_z)
        form.addRow('Roll [rad]:', self.spin_roll)
        form.addRow('Pitch [rad]:', self.spin_pitch)
        form.addRow('Yaw [rad]:', self.spin_yaw)
        layout.addLayout(form)

        # MoveL Button
        self.btn_move = QPushButton('MoveL ausführen')
        self.btn_move.clicked.connect(self.send_moveL)
        layout.addWidget(self.btn_move)

        # Geschwindigkeit Slider
        speed_layout = QHBoxLayout()
        self.ur_speed_slider = QSlider(Qt.Horizontal)
        self.ur_speed_slider.setMinimum(1)
        self.ur_speed_slider.setMaximum(100)
        self.ur_speed_slider.setValue(50)
        self.ur_speed_slider.valueChanged.connect(self.update_speed_label)
        self.ur_speed_label = QLabel('Speed: 0.50')
        speed_layout.addWidget(self.ur_speed_label)
        speed_layout.addWidget(self.ur_speed_slider)
        layout.addLayout(speed_layout)

        # Greifer-Buttons
        grip_layout = QHBoxLayout()
        self.btn_open = QPushButton('Greifer öffnen')
        self.btn_close = QPushButton('Greifer schließen')
        self.btn_open.clicked.connect(self.open_gripper)
        self.btn_close.clicked.connect(self.close_gripper)
        grip_layout.addWidget(self.btn_open)
        grip_layout.addWidget(self.btn_close)
        layout.addLayout(grip_layout)

        # Status
        self.status_label = QLabel('Verbindung: nicht verbunden')
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.resize(400, 300)

    def update_speed_label(self, val):
        speed = val / 100.0
        self.ur_speed_label.setText(f'Speed: {speed:.2f}')

    def connect_to_robot(self):
        try:
            self.sock.connect(('localhost', 30010))
            self.status_label.setText('Verbindung: OK')
            self.status_label.setStyleSheet('color: green')
        except Exception as e:
            self.status_label.setText(f'Fehler: {e}')
            self.status_label.setStyleSheet('color: red')

    def request_current_pose(self):
        # Gelenkwinkel abrufen und Forward Kinematics anwenden (einmalig)
        try:
            cmd = {'command': 'getActualQ'}
            self.sock.sendall(json.dumps(cmd).encode('utf-8'))
            resp = self.sock.recv(4096)
            data = json.loads(resp.decode('utf-8'))
            if 'data' in data and len(data['data']) == 6:
                q = data['data']
                T = self.ur3_model.fkine(q)
                x, y, z = T.t.flatten().tolist()
                roll, pitch, yaw = T.rpy(order='xyz')
                # Werte in SpinBoxes setzen
                for spin, val in [(self.spin_x, x), (self.spin_y, y), (self.spin_z, z),
                                  (self.spin_roll, roll), (self.spin_pitch, pitch), (self.spin_yaw, yaw)]:
                    spin.blockSignals(True)
                    # Clamp to Arbeitsbereich falls nötig
                    spin.setValue(val)
                    spin.blockSignals(False)
        except Exception as e:
            print('Fehler request_current_pose:', e)

    def send_moveL(self):
        pose = [
            self.spin_x.value(),
            self.spin_y.value(),
            self.spin_z.value(),
            self.spin_roll.value(),
            self.spin_pitch.value(),
            self.spin_yaw.value()
        ]
        speed = self.ur_speed_slider.value() / 100.0
        cmd = {'command': 'moveL', 'data': {'pose': pose, 'speed': speed, 'acceleration': 0.3}}
        try:
            self.sock.sendall(json.dumps(cmd).encode('utf-8'))
            resp = self.sock.recv(4096)
            data = json.loads(resp.decode('utf-8'))
            print('MoveL Response:', data)
        except Exception as e:
            print('Fehler send_moveL:', e)

    def open_gripper(self):
        self._send_simple('openGripper')
    def close_gripper(self):
        self._send_simple('closeGripper')
    def _send_simple(self, command):
        try:
            self.sock.sendall(json.dumps({'command': command}).encode('utf-8'))
            resp = self.sock.recv(4096)
            print(f'{command} Response:', resp.decode())
        except Exception as e:
            print(f'Fehler {command}:', e)

    def closeEvent(self, event):
        try: self.sock.close()
        except: pass
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotControlGUI()
    gui.show()
    sys.exit(app.exec_())
