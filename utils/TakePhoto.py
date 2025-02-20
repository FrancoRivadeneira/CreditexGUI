## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import sys 
import cv2
import os
import numpy as np
from pygame.locals import *    ###
import subprocess
import requests

## ROBOT INFO (RASPBERRY REMOTE)
ROBOT_TERRESTRE_IP_ADDR=  "192.168.78.63" #"192.168.1.166""192.168.1.133"
ROBOT_TERRESTRE_USERNAME= "pi"
ROBOT_TERRESTRE_PASSWORD = "raspberry"
puerto = 22  # Puerto por defecto de SSH
ROBOT_TERRESTRE_PORT=8666
ROBOT_TERRESTRE_CAMS_PORT_LIST=[8080,8085]
ROBOT_TERRESTRE_FRAME_RATE=10
ROBOT_TERRESTRE_WIDTH_FRAME_CAM=640
ROBOT_TERRESTRE_HEIGHT_FRAME_CAM=480
ROBOT_TERRESTRE_MIN_VALOR_BATERIA=22
ROBOT_TERRESTRE_RANGO_BATERIA=25-22

class TakePhoto:
    # Señal para actualizar el estado de la conexión en la interfaz principal
    def __init__(self, base_url, save_folder="imagenes"):
        self.base_url = base_url
        self.save_folder = save_folder
        os.makedirs(save_folder, exist_ok=True)  # Crear la carpeta si no existe

  

    def download_photo(self, endpoint, filename="foto.jpg"):
        """Envía una solicitud GET al servidor y guarda la imagen recibida."""
        url = f"{self.base_url}{endpoint}"
        print(url)

        try:
            response = requests.get(url, stream=True)  # Cambiado a GET
            response.raise_for_status()

            # Convertir los bytes de la respuesta en una imagen
            img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            # Guardar la imagen en la carpeta
            file_path = os.path.join(self.save_folder, filename)
            cv2.imwrite(file_path, frame)

            print(f"Frame guardado en {file_path}")
        except requests.exceptions.RequestException as e:
            print(f"Error al descargar el frame: {e}")
