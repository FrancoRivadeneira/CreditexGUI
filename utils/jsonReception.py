## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import time
from pygame.locals import *    ###
import requests                ###

class jsonReception(QThread):
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
                    data = response.json()                                        # Parsear el contenido como JSON
                    data_str = str(data)                                          # Convertir data a string
                    start_index = data_str.find(data_str[0:10])
                    #print(data_str[0:10])
                    if start_index != -1:                                         # Encontrar el índice del comienzo del array
                        start_index = data_str.find('[[', start_index)             # Busca el primer '[' después de "ranges"
                        end_index = data_str.find(']]', start_index)               # Busca el primer ']' después del '['
                        if start_index != -1 and end_index != -1:                 # Extraer el contenido entre los corchetes
                            ranges_content = data_str[start_index + 1:end_index+1]  # Sin incluir los corchetes
                            ranges_content = eval(f"[{ranges_content}]")
                            # Convertir el contenido a una lista de floats
                            self._SIGNAL_data.emit(ranges_content)

            except Exception as e:
                #logging.info("No se detecto Conexion volviendo a intentar")
                self.falseParent._FLAG_LIDAR = False
                time.sleep(0.2)