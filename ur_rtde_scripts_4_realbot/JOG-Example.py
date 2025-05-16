# Dieses Python-Skript steuert einen Universal Robots (UR)-Roboter über die rtde_control-Bibliothek.
# Es ermöglicht die manuelle Bewegung des Roboterwerkzeugs (Tool) mithilfe der Pfeiltasten auf der Tastatur.

#  Dazu nutzen wir die ur_rtde JOG- Funktion (kontinuierliche Bewegung mit konstanter Geschwindigkeit).
# jogStart() startet die "Jogging"-Bewegung 
# https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html#jog-example

import rtde_control
import msvcrt
import time


################### NO TESTED YET ##########################

ROBOT_IP = "192.168.0.3"
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)

# TCP soll bewegt werden
speed_magnitude = 0.05  # Geschwindigkeit in m/s
# Die Roboterbewegung wird als 6D-Vektor definiert (3 lineare + 3 rotatorische Achsen).
speed_vector = [0, 0, 0, 0, 0, 0] # [x, y, z, rx, ry, rz]

try:
    while True:
        if msvcrt.kbhit():  # Prüft, ob eine Taste gedrückt wurde
            key = msvcrt.getch()  # Liest die Taste
            if key == b'\xe0':
                key = msvcrt.getch()
                if key == b'H':  # Pfeil-oben
                    speed_vector = [0, 0, -speed_magnitude, 0, 0, 0]
                elif key == b'P':  # Pfeil-unten
                    speed_vector = [0, 0, speed_magnitude, 0, 0, 0]
                elif key == b'K':  # Pfeil-links
                    speed_vector = [speed_magnitude, 0, 0, 0, 0, 0]
                elif key == b'M':  # Pfeil-rechts
                    speed_vector = [-speed_magnitude, 0, 0, 0, 0, 0]
            elif key == b'q':  # Beenden der Schleife => jogStop (s.u.)
                    break
            else:
                    speed_vector = [0, 0, 0, 0, 0, 0]  # Stopp

            rtde_c.jogStart(speed_vector, rtde_control.C) 
            # startet die "Jogging"-Bewegung (kontinuierliche Bewegung mit konstanter Geschwindigkeit).
            # FEATURE_TOOL bedeutet, dass die Bewegung relativ zum Werkzeugkoordinatensystem erfolgt.

        time.sleep(0.02)  # 20 ms Wartezeit
finally:
    rtde_c.jogStop()
    rtde_c.stopScript()
