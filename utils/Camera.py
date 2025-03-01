# Library for PyQt
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import cv2  # For Camera visualization  ###
import logging  # Libreria para info Logging
import numpy as np
from pygame.locals import *


class Camera(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self, parent=None, id_cam=0):
        super().__init__(None)
        self.idCam = id_cam
        self._FLAG_run = True
        self.falseParent = parent

    def run(self):
        # capture from web cam
        capture_cam = None
        while self._FLAG_run:
            try:
                if capture_cam is None:
                    ip = self.falseParent._VAR_IP_ADDR
                    port = self.falseParent._VAR_CAMS_PORT_LIST[self.idCam-1]
                    path = f"http://{ip}:{port}/?action=stream"
                    capture_cam = cv2.VideoCapture(path)
                    logging.info(f"Camara: {self.idCam}. Connecting to {path}")
                ret, cv_img = capture_cam.read()
                if ret:
                    self.change_pixmap_signal.emit(cv_img)
                    # Grabamos video
                    # self.falseParent._VAR_outVideoWrite_list[self.idCam-1].write(cv_img)
            # shut down capture system
            except Exception as e:
                capture_cam = None
                logging.exception(f"ERROR, cannot capture img of Cam{self.idCam} from path: {path}.. retrying")  # noqa
        try:
            logging.info(f"Releasing handler video of cam {self.idCam}")
            capture_cam.release()
            self.falseParent._VAR_outVideoWrite_list[self.idCam-1] = None
            logging.info(f"camara {self.idCam} released")
        except:
            logging.info(f"thread of cam {self.idCam} suddenly ended")

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._FLAG_run = False
        self.wait()
