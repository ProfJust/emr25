#------------------------------------------------------------------------
# pyQt6_03_ClickMe.py
# Beispiel fuer Push_Button mit Click-Event
# 26.11.2024 by OJ
#------------------------------------------------------------------------
import sys
from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QMainWindow
# Klasse für Schaltflächen (Buttons).
from PyQt6.QtWidgets import QPushButton 
# QSize: Für die Angabe von Größen (z.B. Fenstergröße).
from PyQt6.QtCore import QSize   

# Erstellt eine eigene Fensterklasse, die von QMainWindow erbt.
class MainWindow(QMainWindow):
    def __init__(self):
        # Der Konstruktor (__init__) initialisiert das Hauptfenster.
        QMainWindow.__init__(self)

        # Fenster-Eigenschaften setzen
        self.setMinimumSize(QSize(300, 200))    
        self.setWindowTitle("PyQt button example - pythonprogramminglanguage.com")

        # Erstellt einen Button mit dem Text "Click me".
        self.pybutton = QPushButton('Click me', self)
        self.pybutton.resize(100,32) # Setzt die Größe des Buttons auf 100x32 Pixel.
        self.pybutton.move(50, 50)     
        # Signal mit Slot verbinden
        self.pybutton.clicked.connect(self.clickSlot)         
        # Signal: clicked wird ausgelöst, wenn der Button angeklickt wird.
        # Slot: self.clickSlot ist die Methode, die beim Klick ausgeführt wird.
        # Signal-Slot-Prinzip: In PyQt werden Benutzeraktionen (Signale) mit Funktionen (Slots) verbunden. 

    # Diese Methode wird ausgeführt, wenn der Button geklickt wird.
    def clickSlot(self):
        # gibt den Text "Clicked Pyqt button." in der Konsole aus.
        print('Clicked Pyqt button.')

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit( app.exec() )
