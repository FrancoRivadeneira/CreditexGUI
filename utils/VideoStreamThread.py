from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import time
import pygame
import cv2
pygame.init()
pygame.joystick.init()
class DimensionUpdater(QObject):
    dimensions_changed = pyqtSignal(int, int)


class VideoStreamThread(QThread):
    update_frame = pyqtSignal(QPixmap)

    def __init__(self, url, dimension_updater):
        super().__init__()
        self.stopped = False
        self.cap = cv2.VideoCapture(url)
        self.last_time = time.time()
        self.dimension_updater = dimension_updater
        self.width = 0
        self.height = 0
        self.dimension_updater.dimensions_changed.connect(
            self.update_dimensions)

    def update_dimensions(self, width, height):
        self.width = width
        self.height = height

    def run(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_resized = cv2.resize(frame, (self.width, self.height))
                self.add_fps_text(frame_resized)
                h, w, ch = frame_resized.shape
                qImg = QImage(frame_resized.data, w, h,
                              ch * w, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qImg)
                self.update_frame.emit(pixmap)

        self.cap.release()

    def add_fps_text(self, frame):
        current_time = time.time()
        if (current_time - self.last_time) != 0:
            fps = int(1 / (current_time - self.last_time))
        else:
            fps = 0
        self.last_time = current_time
        cv2.putText(frame, f"FPS: {fps}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def stop(self):
        self.stopped = True
        self.wait()

