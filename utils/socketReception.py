## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
import socket ## For LEDS intensity
import time
from pygame.locals import *    ###
import pickle

class socketReception(QThread):
    _SIGNAL_tramaDatos = pyqtSignal(str)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        
    def run(self):
        while self._FLAG_run:
            try:
                if not(self.falseParent._FLAG_socketConected):
                    self.falseParent._VAR_socketClient= socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                    logging.info("conectando a {} {}".format(self.falseParent._VAR_IP_ADDR,self.falseParent._VAR_PORT))
                    self.falseParent._VAR_socketClient.connect((self.falseParent._VAR_IP_ADDR,self.falseParent._VAR_PORT))
                    self.falseParent._FLAG_socketConected=True
                tramaDatos = self.falseParent._VAR_socketClient.recv(4096)
                tramaDatos = pickle.loads(tramaDatos)
                #tramaDatos = tramaDatos.decode("latin-1")
                self._SIGNAL_tramaDatos.emit(tramaDatos)
            except:
                logging.info("Se perdió conexión. Volviendo a conectar con el socket ")
                self.falseParent._FLAG_socketConected=False
                time.sleep(1)
        try:
            self.falseParent._VAR_socketClient.close()
            logging.info("socket con el robot cerrado!")
        except:
            logging.info("El socket ya se habia cerrado previamente..")
            
    def stop(self):
        self._FLAG_run=False
        self.wait()    
