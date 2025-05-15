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
input("Weiter? -> Taste")
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

        # Alle Achsen lesen und ausgeben        
        x = joystick.get_axis(0)
        y = joystick.get_axis(1)
        a2 = joystick.get_axis(2)
        # a3 = joystick.get_axis(3) entspricht a2
        a4 = joystick.get_axis(4)
        print(f"X: {x:.2f}, Y: {y:.2f}, 2: {a2:.2f},4: {a4:.2f}", end=" ")

        # Alle Buttons holen und ausgeben
        for i in range(joystick.get_numbuttons()):
            print(joystick.get_button(i), end=" ")
        print("-------")
    
    except KeyboardInterrupt:
        print("KeyboardInterrupt:")
        sys.exit()