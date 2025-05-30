# Demo Joystik mit Webots emr25_world_2_IK_Corke.wbt
# hama Gamepad an USB anschliessen
# OJ 15.05.25
# Wenn das Modul außerhalb des aktuellen Projektverzeichnisses liegt, musst du Python mitteilen, dass es auch in anderen Verzeichnissen nach Modulen suchen soll. 
# Hier wird das gewünschte Verzeichnis zur Suchliste (sys.path) hinzugefügt, sodass Python das Modul finden und importieren kann
# Setze ein kleines r vor den String, damit Python die Backslashes nicht als Escape-Zeichen interpretiert
import sys
import os
# fixe Version des Pfades, bei jedem PC anders
#path2rtde_files = r"C:\Users\RoboAdmin\mySciebo\_EMR25\labor\emr25\ur_rtde_webot_control_lib" 

# Besser vom aktuellen Arbeitsverzeichnis ausgehen
path2rtde_files = os.getcwd() + r"\ur_rtde_webot_control_lib"
print(path2rtde_files)
sys.path.insert(1, path2rtde_files)
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from robotiq_gripper_control import RobotiqGripper
import time


### MAKE A BEEP ###
import pygame # ggf. pip install pygame
print(" THIS SOFTWARE IS UNDER HEAVY CONSTRUCTION ")
pygame.mixer.init()
sound = pygame.mixer.Sound('owin31.wav')
sound.play()


UR3_IP = "127.0.0.1"

rtde_r = RTDEReceiveInterface(UR3_IP)
rtde_c = RTDEControlInterface(UR3_IP)
gripper = RobotiqGripper(rtde_c)

# Gripper Aktionen
gripper.activate()
#gripper.set_force(50)
#gripper.set_speed(100)
griffweite = 5

import pygame  # ggf. pip install pygame bzw. py -m pip install pygame
# https://www.pygame.org/docs/ref/joystick.html
import sys

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Infos holen
print("Anzahl der Achsen:", joystick.get_numaxes() )
print("Anzahl der Buttons:", joystick.get_numbuttons() )
print(" ===== >>>>   Analog Modus am Gamepad einschalten !!")
# input("Weiter? -> Taste")

nothalt = True
print(" Not-Halt ist eingeschaltet, \n  Start mit |>  - Play-Button B10, \n  Nothalt auslösen mit B9")

# rumble funktioniert beim HAMA nicht
# joystick.rumble(1, 50, 0) #rumble(low_frequency, high_frequency, duration) -> bool
# input("Weiter? -> Taste")
# joystick.stop_rumble()

# Schleife zum fortwährenden Auslesen des Gamepads
while True:
    try:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        # --- Alle Achsen lesen und ausgeben ---        
        x = joystick.get_axis(0)
        # Deadzone für Analogsticks (vermeidet Rauschen):
        if abs(x) < 0.1: x = 0.0

        y = joystick.get_axis(1)
        # Deadzone für Analogsticks (vermeidet Rauschen):
        if abs(y) < 0.1: y = 0.0

        a2 = joystick.get_axis(2)
        if abs(a2) < 0.1: a2 = 0.0

        # a3 = joystick.get_axis(3) entspricht a2
        a4 = joystick.get_axis(4)
        if abs(a4) < 0.1: a4 = 0.0         

        if joystick.get_button(8):
            nothalt = True
            print(" Nothalt ")
        if joystick.get_button(9):
            nothalt = False
            print(" Nothalt aus")

        if not nothalt:
            print(f"X: {x:.2f}, Y: {y:.2f}, 2: {a2:.2f},4: {a4:.2f}", end=" ")
            # --- Alle Buttons holen und ausgeben ---
            for i in range(joystick.get_numbuttons()):
                print(joystick.get_button(i), end=" ")
            print("-------")

            # --- Gripper Aktionen ---
            if joystick.get_button(5):
                gripper.close()        
            if joystick.get_button(7):
                gripper.open()
    
            # get actual pose, joint angles in RAD
            actual_q = rtde_r.getActualQ() 
            # new pose the robot should go to
            new_q = actual_q
            new_q[0] = new_q[0] + x * 0.5   # Joy Axis 0     shoulder_pan_joint
            new_q[1] = new_q[1] + y * 0.5   # Joy Axis 1     shoulder_lift_joint
            new_q[2] = new_q[2] + a4 * 0.5  # Joy Axis 4    elbow_joint
            new_q[3] = new_q[3] + a2 * 0.5  # Joy Axis 1   wrist_1_joint
            new_q[4] = new_q[4] + joystick.get_button(0) * 0.5  # wrist_2_joint
            new_q[4] = new_q[4] - joystick.get_button(2) * 0.5
            new_q[5] = new_q[5] + joystick.get_button(1) * 0.5  # wrist_3_joint
            new_q[5] = new_q[5] - joystick.get_button(3) * 0.5
            resp = rtde_c.moveJ(new_q)

    except KeyboardInterrupt:
        print("KeyboardInterrupt:")
        rtde_c.disconnect()
        rtde_r.disconnect()
        sys.exit()
    