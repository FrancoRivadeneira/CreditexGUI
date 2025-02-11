from tcam import TCam
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import time
import pygame
import cv2
pygame.init()
pygame.joystick.init()
import numpy as np
import base64



class VideoThermalThread(QThread):
    update_frame = pyqtSignal(QPixmap)

    def __init__(self, dimension_updater):
        super().__init__()
        global is_runing
        self.tcam=TCam()

        x = self.tcam.connect('192.168.0.30')
        print(x["status"])
        if x["status"]=='disconnected':
            self.tcam.shutdown()
            raise ConnectionError

        self.tcam.start_stream()



        self.frame_count=0
        #self.start=perf_counter()
        self.stopped = False

        self.last_time = time.time()
        self.dimension_updater = dimension_updater
        self.width = 0
        self.height = 0
        self.dimension_updater.dimensions_changed.connect(self.update_dimensions)

    def update_dimensions(self, width, height):
        self.width = width
        self.height = height

    def convert(self, img):
        if img is None:
            return None

        dimg = base64.b64decode(img['radiometric'])
        ra = np.frombuffer(dimg, dtype=np.uint16).reshape(120, 160)

        dtel = base64.b64decode(img['telemetry'])
        rt = np.frombuffer(dtel, dtype=np.uint16)

        TLinear_resolution_flag = rt[209]
        TLinear_resolution = 0.01 if TLinear_resolution_flag else 0.1
        a = ra * TLinear_resolution - 273.15

        return a
    def __del__(self):
        self.tcam.disconnect()

    def run(self):
        while not self.stopped:
            while (frame:=self.tcam.get_frame()) is not None:
                temperature_data = self.convert(frame)


                # print(temperature_data)

                #frame_count += 1

                # fps = frame_count / (end - self.start)
                # print(f'FPS: {fps:.2f}')

                # Normalize temperature data to range 0-255
                temperature_data_normalized = (temperature_data - np.min(temperature_data)) / (
                    np.max(temperature_data) - np.min(temperature_data)) * 255
                temperature_data_normalized = temperature_data_normalized.astype(
                    np.uint8)

                # Apply a colormap
                colormap = cv2.applyColorMap(
                    temperature_data_normalized, cv2.COLORMAP_HOT)


                frame_resized = cv2.resize(colormap, (self.width, self.height))
                self.add_fps_text(frame_resized)
                h, w, ch = frame_resized.shape
                qImg = QImage(frame_resized.data, w, h,
                                ch * w, QImage.Format_BGR888)
                pixmap = QPixmap.fromImage(qImg)
                self.update_frame.emit(pixmap)
                if not is_runing:
                    self.tcam.shutdown()


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
