## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
import time
from pygame.locals import *    ###
import requests                ###

class cppjsonReception(QThread):
    _SIGNAL_data = pyqtSignal(list)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
    def run(self): ## Hilo
        # URL del JSON
        #url = 'http://192.168.68.102:5000/lidar'
        url = 'http://'+"192.168.1.4"+":"+"8080"
        while self._FLAG_run:
            try:        
                response = requests.get(url)
                if response.status_code == 200:                                   # Verificar que la solicitud fue exitosa (código 200)
                    self.falseParent._FLAG_LIDAR = True
                    data = response.text                                       # Parsear el contenido como JSON
                    lidar_data_start = data.find('[')  # Encuentra el inicio de la lista
                    lidar_data_end = data.rfind(']')   # Encuentra el final de la lista
                    if lidar_data_start != -1 and lidar_data_end != -1:
                        # Extraer la cadena que está dentro de los corchetes
                        lidar_points_str = data[lidar_data_start:lidar_data_end + 1]
                        ranges_content = eval(f"[{lidar_points_str}]")
                        ranges_content = ranges_content[0]
                        self._SIGNAL_data.emit(ranges_content)
                        #lidar_points = eval(f"[{lidar_points}]")
                        #print(f"Lidar Points: {type(ranges_content[-1][0])}")
            except Exception as e:
                logging.info("No se detecto Conexion volviendo a intentar")
                self.falseParent._FLAG_LIDAR = False
                time.sleep(0.2)
