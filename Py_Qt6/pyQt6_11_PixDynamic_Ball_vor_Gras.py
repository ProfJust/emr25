#!/usr/bin/env python3
# pyQt6_11_PixDynamic_Ball_vor_Gras.py
#---------------------------------------
# Erstellt ein Fenster mit Gras vor dem ein Ball fliegt
# -----------------------------------------
# last edited by OJ am 26.11.2024 
#


import sys
from PyQt6.QtCore import (Qt, QTimer, QRect)
from PyQt6.QtWidgets import (QWidget, QPushButton, QApplication, QLabel)
from PyQt6.QtGui import QPainter, QColor, QFont
from PyQt6.QtGui import QPixmap
import os


class Ui(QWidget):
    # statische Klassenvariablen
    pos_x = 200
    pos_y = 20
    speed_x = -10
    speed_y = +12

    def __init__(self):  # Konstrukor
        # Konstruktor der Elternklasse aufrufen
        super(Ui, self).__init__()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(20)

        # In Python benötigen wir immer den gesamten Dateipfad um eine Datei zu öffen, 
        # falls die Datei nicht im selben Verzeichnis liegt
        # ==> Lösung 1: 
        # in der Windows-Power-Shell in das Verzeichnis springen und
        # > cd 'C:\Users\RoboAdmin\mySciebo\_SRO\_GitHub_LaborPC\SRO\Py_Qt6\'
        # ausführen
        # > python .\pyQt6_11_PixDynamic_Ball_vor_Gras.py
        #
        # Hier aus dem Debugger funktioniert es nicht (Fenster ohen Gras und ohne Ball)
        # Die Bilddateien müssen im selben Ordner liegen !!
        
        # Lösung 2:
        # mit os kann man den Dateipfad bekommen um den kompletten Pfad zusammen zu bauen
        # Bsp: 
        """#https://stackoverflow.com/questions/5137497/find-the-current-directory-and-files-directory
        print("Path at terminal when executing this file")
        print(os.getcwd() + "\n")
        print("This file path, relative to os.getcwd()")
        print(__file__ + "\n")
        print("This file full path (following symlinks)")
        full_path = os.path.realpath(__file__)
        print(full_path + "\n")
        print("This file directory and name")
        path, filename = os.path.split(full_path)
        print(path + ' --> ' + filename + "\n")
        print("This file directory only")
        print(os.path.dirname(full_path)) """
                
        # This file directory and name of this file
        full_path = os.path.realpath(__file__)
        # This file directory only
        dir_path = os.path.dirname(full_path)
        print("directory only", dir_path)
         # join two paths
        self.path_gras = dir_path  + "\\pics\\gras.jpg"
        print("path Gras", self.path_gras)
        self.path_ball = dir_path  + "\\pics\\ball_transparent.png"
        print("path Ball", self.path_ball)
        self.initUI()

    def initUI(self):
        # UI-Fenster Konfigurieren
        self.setGeometry(30, 30, 600, 600)
        self.setWindowTitle('Qt - Painter')
        self.show()

    def paintEvent(self, event):
        p = QPainter()
        p.begin(self)
        self.drawFunc(event, p)
        p.end()

    def drawFunc(self, event, p):
        # Hintergrund mit Pixmap
        
        # pix = QPixmap("pics/gras.jpg")
        pix = QPixmap(self.path_gras)
        p.drawPixmap(self.rect(), pix)

        # bewegtes Rechteck zeichnen mit Pixmap
        pix2 = QPixmap(self.path_ball) # PNG mit Transparenz
        # pix2 = QPixmap(self.abs_path_ball)
        target = QRect(self.pos_x, self.pos_y, 50, 50)  # import QRect
        p.drawPixmap(target, pix2)

    def update(self):
        self.pos_x = self.pos_x + self.speed_x
        self.pos_y = self.pos_y + self.speed_y
        # Pruefe Rand
        if self.pos_x < 0:
            self.speed_x = -self.speed_x
        if self.pos_x > 600:
            self.speed_x = -self.speed_x
        if self.pos_y < 0:
            self.speed_y = -self.speed_y
        if self.pos_y > 600:
            self.speed_y = -self.speed_y
        self.repaint()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = Ui()
    sys.exit(app.exec())
