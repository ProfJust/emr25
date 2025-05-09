import sys
import time
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QProgressBar
import pygame
import rtde_control
import rtde_receive
import robotiq_gripper

ROBOT_IP = "192.168.0.51"
#Notaus Label
class ClickableLabel(QLabel):
    clicked = pyqtSignal()

    def mousePressEvent(self, event):
        self.clicked.emit()

class RobotControlThread(QThread):
    updateValues = pyqtSignal(float, float, float, float, float, float)
    updateGripperState = pyqtSignal(str, bool)

    def __init__(self, rtde_r, rtde_c, joystick):
        super().__init__()
        self.rtde_r = rtde_r
        self.rtde_c = rtde_c
        self.joystick = joystick
        self.running = True
        self.paused = False
        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(ROBOT_IP, 63352)
        self.gripper.activate()
        self.speed_magnitude = 0.05 #speed X,Y,Z
        self.speed_magnitude_r = 0.20 #speed roll,pitch,yaw

    def run(self):
        while self.running:
            try:
                pygame.event.pump()

                if self.joystick.get_button(7):  # Start button
                    print("Emergency stop activated")
                    self.paused = True
                    self.rtde_c.jogStop()

                if self.joystick.get_button(6):  # Back button
                    print("Robot activated")
                    self.paused = False

                if not self.paused:
                    # Use Left stick for translational movement
                    self.state_x = self.joystick.get_axis(0)  # Left stick horizontal
                    self.state_y = -self.joystick.get_axis(1)  # Left stick vertical (inverted axis)
                    self.state_z = 0  #state_z init

                    # Handle Z-axis movement with LB and RB buttons
                    if self.joystick.get_button(4):  # LB button
                        self.state_z -= 1  # Move in negative Z direction
                    if self.joystick.get_button(5):  # RB button
                        self.state_z += 1  # Move in positive Z direction

                    # Use right stick for rotational movement
                    self.state_yaw = self.joystick.get_axis(2)  # Right stick horizontal
                    self.state_pitch = -self.joystick.get_axis(3)  # Right stick vertical
                    self.state_roll = 0  # Reset roll movement

                    # Use D-Pad for roll control
                    hat_x, hat_y = self.joystick.get_hat(0)
                    if hat_y > 0:  # Up D-Pad
                        self.state_roll += 1
                    elif hat_y < 0:  # Down D-Pad
                        self.state_roll -= 1

                    # Handle gripper actions
                    if self.joystick.get_button(0):  # A button
                        print("Gripper opening...")
                        self.gripper.move_and_wait_for_pos(0, 255, 255)
                        self.updateGripperState.emit("A Button: Pressed", self.gripper.is_open())
                    elif self.joystick.get_button(1):  # B button
                        print("Gripper closing...")
                        self.gripper.move_and_wait_for_pos(255, 255, 255)
                        self.updateGripperState.emit("B Button: Pressed", self.gripper.is_open())

                    # Emit signal to update bars
                    self.updateValues.emit(
                        self.state_x, self.state_y, self.state_z, self.state_roll, self.state_pitch, self.state_yaw
                    )

                    speed_vector = self.calculate_speed_vector()
                    self.rtde_c.jogStart(speed_vector)
                    time.sleep(0.01)  # Adjusted frequency for control updates

            except Exception as e:
                print(f"Error in run loop: {e}")

    def stop(self):
        self.running = False
        self.rtde_c.jogStop()

    def calculate_speed_vector(self):
        return [
            self.state_x * self.speed_magnitude,
            self.state_y * self.speed_magnitude,
            self.state_z * self.speed_magnitude,
            self.state_roll * self.speed_magnitude_r,
            self.state_pitch * self.speed_magnitude_r,
            self.state_yaw * self.speed_magnitude_r
        ]

class XBoxController(QWidget):
    def __init__(self, joystick):
        super().__init__()
        self.joystick = joystick
        self.rtde_c = None
        self.rtde_r = None
        self.robot_thread = None
        self.initUI()
        self.initRobot()

    def initUI(self):
        self.setWindowTitle("Xbox Controller Data")
        self.setStyleSheet("background-color: #2e2e2e;")

        self.x_label = QLabel("X: --", self)
        self.x_bar = QProgressBar(self)
        self.y_label = QLabel("Y: --", self)
        self.y_bar = QProgressBar(self)
        self.z_label = QLabel("Z: --", self)
        self.z_bar = QProgressBar(self)

        self.roll_label = QLabel("Roll: --", self)
        self.roll_bar = QProgressBar(self)
        self.pitch_label = QLabel("Pitch: --", self)
        self.pitch_bar = QProgressBar(self)
        self.yaw_label = QLabel("Yaw: --", self)
        self.yaw_bar = QProgressBar(self)

        self.lbl_pose_complete = QLabel("Pose: --", self)

        self.emergency_stop_label = ClickableLabel("NOTAUS_BUTTON", self)
        self.emergency_stop_label.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.emergency_stop_label.clicked.connect(self.handle_emergency_stop)

        self.A_button_label = QLabel("A Button: Released", self)
        self.B_button_label = QLabel("B Button: Released", self)
        self.gripper_label = QLabel("Gripper Closed", self)

        font_style = "color: white; font-family: 'Times New Roman'; font-size: 40px;"
        for label in [self.x_label, self.y_label, self.z_label, self.roll_label,
                      self.pitch_label, self.yaw_label, self.lbl_pose_complete,
                      self.A_button_label, self.B_button_label, self.gripper_label]:
            label.setStyleSheet(font_style)

        for bar in [self.x_bar, self.y_bar, self.z_bar, self.roll_bar, self.pitch_bar, self.yaw_bar]:
            bar.setRange(-100, 100)
            bar.setStyleSheet("QProgressBar { background-color: #444; color: white; }"
                              "QProgressBar::chunk { background-color: #00bfff; }")

        layout = QVBoxLayout()
        for label, bar in [(self.x_label, self.x_bar), (self.y_label, self.y_bar),
                           (self.z_label, self.z_bar), (self.roll_label, self.roll_bar),
                           (self.pitch_label, self.pitch_bar), (self.yaw_label, self.yaw_bar)]:
            layout.addWidget(label)
            layout.addWidget(bar)

        layout.addWidget(self.A_button_label)
        layout.addWidget(self.B_button_label)
        layout.addWidget(self.gripper_label)
        layout.addWidget(self.lbl_pose_complete)
        layout.addWidget(self.emergency_stop_label)

        self.setLayout(layout)
        self.setGeometry(100, 100, 400, 600)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateJoystickData)
        self.timer.start(10)  # Update every 10ms

    def initRobot(self):
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
            self.robot_thread = RobotControlThread(self.rtde_r, self.rtde_c, self.joystick)
            self.robot_thread.updateValues.connect(self.updateBars)
            self.robot_thread.updateGripperState.connect(self.updateGripperInfo)
            self.robot_thread.start()
        except Exception as e:
            print(f"Failed to initialize robot: {e}")
            sys.exit(1)

    def handle_emergency_stop(self):
        print("Notaus betätigt")
        if self.robot_thread:
            self.robot_thread.stop()
            self.robot_thread = None
        self.close()
    #Bar graph
    def updateBars(self, x, y, z, roll, pitch, yaw):
        self.x_bar.setValue(int(x * 100))
        self.y_bar.setValue(int(y * 100))
        self.z_bar.setValue(int(z * 100))
        self.roll_bar.setValue(int(roll * 100))
        self.pitch_bar.setValue(int(pitch * 100))
        self.yaw_bar.setValue(int(yaw * 100))
        
        self.x_label.setText(f"X: {x:.2f}")
        self.y_label.setText(f"Y: {y:.2f}")
        self.z_label.setText(f"Z: {z:.2f}")
        self.roll_label.setText(f"Roll: {roll:.2f}")
        self.pitch_label.setText(f"Pitch: {pitch:.2f}")
        self.yaw_label.setText(f"Yaw: {yaw:.2f}")

    def updateGripperInfo(self, button_status, gripper_open):
        if "A" in button_status:
            self.A_button_label.setText("A Button: Pressed")
            self.B_button_label.setText("B Button: Released")
        elif "B" in button_status:
            self.B_button_label.setText("B Button: Pressed")
            self.A_button_label.setText("A Button: Released")

        # Update gripper state
        self.gripper_label.setText("Gripper " + ("Opened" if gripper_open else "Closed"))

    def updateJoystickData(self):
        try:
            pygame.event.pump()
            current_pose = self.rtde_r.getActualTCPPose()
            pose_text = f"Pose: x={current_pose[0]:.3f}, y={current_pose[1]:.3f}, z={current_pose[2]:.3f}, " \
                        f"rx={current_pose[3]:.3f}, ry={current_pose[4]:.3f}, rz={current_pose[5]:.3f}"
            self.lbl_pose_complete.setText(pose_text)

        except Exception as e:
            print(f"Error updating joystick data: {e}")

    def closeEvent(self, event):
        if self.robot_thread:
            self.robot_thread.stop()
            self.robot_thread.wait()
        pygame.quit()
        event.accept()

def init_controller():
    pygame.init()
    if pygame.joystick.get_count() < 1:
        raise Exception("No controller detected! Please connect an Xbox controller.")
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to controller: {joystick.get_name()}")
    return joystick

def main():
    try:
        joystick = init_controller()

        app = QApplication(sys.argv)
        window = XBoxController(joystick)
        window.show()

        sys.exit(app.exec_())
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()