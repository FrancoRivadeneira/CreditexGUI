# Library for PyQt
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import time
from pygame.locals import *
import requests


class jsonReception(QThread):
    _SIGNAL_data = pyqtSignal(list)

    def __init__(self, parent):
        super().__init__(None)
        self._FLAG_run = True
        self.falseParent = parent

    def run(self):  # Hilo
        # URL del JSON
        # url = 'http://192.168.68.201:5000/lidar'
        url = 'http://'+"192.168.1.4"+":"+"8080"
        while self._FLAG_run:
            try:
                response = requests.get(url)
                # Verificar que la solicitud fue exitosa (código 200)
                if response.status_code == 200:
                    self.falseParent._FLAG_LIDAR = True
                    # Parsear el contenido como JSON
                    data = response.json()
                    # Convertir data a string
                    data_str = str(data)
                    start_index = data_str.find(data_str[0:10])
                    # print(data_str[0:10])
                    if start_index != -1:                                         # Encontrar el índice del comienzo del array
                        # Busca el primer '[' después de "ranges"
                        start_index = data_str.find('[[', start_index)
                        # Busca el primer ']' después del '['
                        end_index = data_str.find(']]', start_index)
                        if start_index != -1 and end_index != -1:                 # Extraer el contenido entre los corchetes
                            # Sin incluir los corchetes
                            ranges_content = data_str[start_index +
                                                      1:end_index+1]
                            ranges_content = eval(f"[{ranges_content}]")
                            # Convertir el contenido a una lista de floats
                            self._SIGNAL_data.emit(ranges_content)

            except Exception as e:
                # logging.info("No se detecto Conexion volviendo a intentar")
                self.falseParent._FLAG_LIDAR = False
                time.sleep(0.2)
