from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import socket
import logging
import pygame
import cv2
import numpy as np
import base64   
from concurrent.futures import ThreadPoolExecutor
pygame.init()
pygame.joystick.init()


class VideoClient(QThread):
    change_pixmap_signal = pyqtSignal(bytes)

    def __init__(self, directory, idCam, dimension_updater, ip, port):
        super().__init__()
        self.idCam = idCam

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.BUFF_SIZE = 65536
        self.host_ip = ip
        self.port = port
        self.message = b'Hi'
        self.executor = ThreadPoolExecutor(max_workers=2)
        self.dimension_updater = dimension_updater
        self.width = 0
        self.height = 0

        self.client_socket.connect((self.host_ip, self.port))

        # Connect the signal to the slot (function) that will handle dimension updates
        self.dimension_updater.dimensions_changed.connect(
            self.update_dimensions)

    def update_dimensions(self, width, height):
        self.width = width
        self.height = height

    def run(self):
        try:

            while True:

                self.client_socket.sendto(
                    self.message, (self.host_ip, self.port))
                packet, _ = self.client_socket.recvfrom(self.BUFF_SIZE)
                data = base64.b64decode(packet, ' /')
                npdata = np.fromstring(data, dtype=np.uint8)
                frame = cv2.imdecode(npdata, 1)
                frame = cv2.resize(frame, (self.width, self.height))

                is_success, buffer = cv2.imencode(".jpg", frame)
                if is_success:
                    # Emit the bytes signal
                    print("todo bien 3")
                    self.change_pixmap_signal.emit(buffer.tobytes())
        except Exception as e:
            logging.error("Error during video streaming: {}".format(e))

    def stop(self):
        self.client_socket.close()
        self.wait()
