## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import cv2 ## For Camera visualization  ###
import logging ## Libreria para info Logging
import numpy as np             ###
from pygame.locals import *    ###

class Camera(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    def __init__(self,parent=None,id_cam=0):
        super().__init__(None)
        self.idCam=id_cam
        self._FLAG_run=True
        self.falseParent=parent

    def run(self):
        # capture from web cam
        capture_cam=None
        while self._FLAG_run:
            try:
                if capture_cam==None:
                    path="http://{}:{}/?action=stream".format(self.falseParent._VAR_IP_ADDR,self.falseParent._VAR_CAMS_PORT_LIST[self.idCam-1])
                    capture_cam = cv2.VideoCapture(path)
                    logging.info("Camara: {}. Connecting to {}".format(self.idCam,path))
                ret, cv_img = capture_cam.read()
                if ret:
                    self.change_pixmap_signal.emit(cv_img)
                    ## Grabamos video
                    self.falseParent._VAR_outVideoWrite_list[self.idCam-1].write(cv_img)
            # shut down capture system
            except:
                capture_cam=None
                logging.info("ERROR,  cannot capture img of Cam{} from path: {}.. retrying".format(self.idCam,path))
        try:
            logging.info("Releasing handler video of cam {}".format(self.idCam))
            capture_cam.release()
            self.falseParent._VAR_outVideoWrite_list[self.idCam-1]=None
            logging.info("camara {} released".format(self.idCam))
        except:
            logging.info("thread of cam {} suddendly ended".format(self.idCam))
        
    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._FLAG_run = False
        self.wait()
