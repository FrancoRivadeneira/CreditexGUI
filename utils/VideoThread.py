from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import pygame
import cv2
pygame.init()
pygame.joystick.init()
is_taking_pictures = False  # determina si empezo el proceso de captura de imagenes


class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(bytes)

    data_received = pyqtSignal(str)

    global ini
    global distancia_abs

    # Se inicia la clase Video Threat
    def __init__(self, directory, camera_id, dimension_updater):
        super().__init__()
        self.is_running = True
        self.directory = directory
        self.camera_id = camera_id
        self.dimension_updater = dimension_updater
        self.width = 0
        self.height = 0

        # Connect the signal to the slot (function) that will handle dimension updates
        self.dimension_updater.dimensions_changed.connect(
            self.update_dimensions)

    def update_dimensions(self, width, height):
        self.width = width
        self.height = height

    def run(self):
        global width
        global height
        global posicion_label
        global distancia_abs

        # Se define la camara que se desea utilizar
        cap = cv2.VideoCapture(self.camera_id)

        # Se determina el tamanio de captura de imagen definido por las variables locales

        # Se inicia el primer threat
        while self.is_running:

            # Capture a frame
            ret, frame = cap.read()

            if ret:
                frame = cv2.resize(frame, (self.width, self.height))
                is_success, buffer = cv2.imencode(".jpg", frame)
                if is_success:
                    # Emit the bytes signal
                    self.change_pixmap_signal.emit(buffer.tobytes())

                if is_taking_pictures:
                    # Se determina el tamano en el que se guardaran las imagenes.
                    frame = cv2.resize(frame, (1920, 1080))

        cap.release()

    def stop(self):
        self.is_running = False

