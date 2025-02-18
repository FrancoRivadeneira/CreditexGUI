from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import cv2 ## For Camera visualization  ###
import logging ## Libreria para info Logging
import numpy as np             ###
import requests  ## Para realizar la solicitud HTTP GET
from pygame.locals import *    ###

class PTZ(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    
    def __init__(self, parent=None):
        super().__init__(None)
        self._FLAG_run = True
        self.falseParent = parent
        self.url = "http://10.100.110.26:1234/ptz_stream"
        self.control_url = "http://10.100.110.26:1234"
    
    def run(self):
        try:
            response = requests.get(self.url, stream=True)
            if response.status_code == 200:
                logging.info(f"Conectado a {self.url}")
                bytes_data = b''
                for chunk in response.iter_content(chunk_size=1024):
                    if not self._FLAG_run:
                        break
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        cv_img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if cv_img is not None:
                            self.change_pixmap_signal.emit(cv_img)
            else:
                logging.error(f"Error al conectar con {self.url}, código de estado: {response.status_code}")
        except Exception as e:
            logging.error(f"Error en la transmisión de video: {e}")
    
    def stop(self):
        """Detiene el hilo de captura de video"""
        self._FLAG_run = False
        self.wait()
    
    def send_control_command(self, command):
        """Envía un comando POST vacío al endpoint especificado"""
        try:
            endpoint = f"{self.control_url}/{command}"
            response = requests.post(endpoint)
            if response.status_code == 200:
                logging.info(f"Comando {command} enviado con éxito")
            else:
                logging.error(f"Error al enviar comando {command}, código de estado: {response.status_code}")
        except Exception as e:
            logging.error(f"Error al enviar comando {command}: {e}")
    
    def zoom_in(self):
        self.send_control_command("zoom_in")

    
    def zoom_out(self):
        self.send_control_command("zoom_out")
    
    def move_right(self):
        self.send_control_command("right")
    
    def move_left(self):
        self.send_control_command("left")
    
    def move_up(self):
        self.send_control_command("up")
    
    def move_down(self):
        self.send_control_command("ptz_down")
        print("Bajando PTZ")
