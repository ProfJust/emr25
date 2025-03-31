# PyQtGUI.py
# PyQtGUI.py
import sys
import socket
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout

class RobotGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 300, 200)
        layout = QVBoxLayout()
        
        # Schaltflächen für Bewegungen
        self.move_up_button = QPushButton('Arm hoch')
        self.move_down_button = QPushButton('Arm runter')
               
        # Verbinden der Schaltflächen mit Funktionen
        self.move_up_button.clicked.connect(self.move_arm_up)
        self.move_down_button.clicked.connect(self.move_arm_down)
               
        layout.addWidget(self.move_up_button)
        layout.addWidget(self.move_down_button)
              
        self.setLayout(layout)
        self.show()

    def move_arm_up(self):
        # Hier kommt der Code, um den Arm hoch zu bewegen
        print("Arm hoch bewegen")
        # Verbindung herstellen und Befehl senden
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 12345))
        print(sock)
        sock.send(b'move_up')
        sock.close()
        
    def move_arm_down(self):
        # Hier kommt der Code, um den Arm runter zu bewegen
        print("Arm runter bewegen")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 12345))
        print(sock)
        sock.send(b'move_down')
        sock.close()
       
   
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotGUI()
    sys.exit(app.exec_())
