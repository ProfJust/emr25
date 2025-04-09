import sys
import json
import socket

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
# ModuleNotFoundError: No module named 'PyQt5' =>   pip install PyQt5

class RobotControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.current_angles = [0.0] * 6  # Aktuelle Gelenkwinkel speichern
        self.initUI()
        self.connect_to_robot()

    def initUI(self):
        self.setWindowTitle('UR3e Joint Control')
        layout = QVBoxLayout()

        # Slider für jedes Gelenk
        self.joint_sliders = []
        self.value_labels = []
        joint_ranges = [
            (-3.14, 3.14),  # shoulder_pan_joint
            (-3.14, 0.0),   # shoulder_lift_joint (physikalische Limits)
            (-3.14, 3.14),  # elbow_joint
            (-3.14, 3.14),  # wrist_1_joint
            (-3.14, 3.14),  # wrist_2_joint
            (-3.14, 3.14)   # wrist_3_joint
        ]

        for i, (min_val, max_val) in enumerate(joint_ranges):
            group = QGroupBox(f'Gelenk {i+1} ({["Pan", "Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"][i]})')
            vbox = QVBoxLayout()
            
            lbl = QLabel('0.00 rad')
            self.value_labels.append(lbl)
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(min_val * 100))
            slider.setMaximum(int(max_val * 100))
            slider.valueChanged.connect(
                lambda value, idx=i: self.slider_changed(value, idx)
            )
            
            vbox.addWidget(lbl)
            vbox.addWidget(slider)
            group.setLayout(vbox)
            self.joint_sliders.append(slider)
            layout.addWidget(group)

        # Status-Anzeige
        self.status_label = QLabel("Verbindung wird hergestellt...")
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.setMinimumSize(500, 700)

    def connect_to_robot(self):
        try:
            self.sock.connect(('localhost', 30010))
            self.status_label.setText("Verbunden mit Roboter")
            self.status_label.setStyleSheet("color: green")
            
            # Initialwerte vom Roboter abfragen
            self.request_current_angles()
            
            # Auto-Update Timer
            self.update_timer = QTimer()
            self.update_timer.timeout.connect(self.request_current_angles)
            self.update_timer.start(500)  # Alle 500ms aktualisieren
            
        except socket.error as e:
            self.status_label.setText(f"Verbindungsfehler: {str(e)}")
            self.status_label.setStyleSheet("color: red")

    def request_current_angles(self):
        try:
            cmd = json.dumps({"command": "getActualQ"})
            self.sock.sendall(cmd.encode('utf-8'))
            response = self.sock.recv(4096)
            data = json.loads(response.decode())
            
            if 'data' in data:
                self.current_angles = data['data']
                self.update_sliders()
                
        except Exception as e:
            print("Abfragefehler:", str(e))

    def update_sliders(self):
        # Blockiere Slider-Events während der Aktualisierung
        for slider in self.joint_sliders:
            slider.blockSignals(True)
            
        for i, angle in enumerate(self.current_angles):
            self.joint_sliders[i].setValue(int(angle * 100))
            self.value_labels[i].setText(f"{angle:.2f} rad")
            
        for slider in self.joint_sliders:
            slider.blockSignals(False)

    def slider_changed(self, value, joint_index):
        rad_value = value / 100.0
        self.current_angles[joint_index] = rad_value
        self.send_joint_command()

    def send_joint_command(self):
        command = {
            "command": "moveJ",
            "data": self.current_angles.copy()
        }
        
        try:
            self.sock.sendall(json.dumps(command).encode('utf-8'))
            response = self.sock.recv(4096)
            print("Serverantwort:", response.decode())
        except Exception as e:
            print("Kommunikationsfehler:", str(e))

    def closeEvent(self, event):
        self.sock.close()
        self.update_timer.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotControlGUI()
    ex.show()
    sys.exit(app.exec_())
