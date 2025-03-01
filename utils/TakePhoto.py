# Library for PyQt
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import cv2
import os
import numpy as np
from pygame.locals import *
import requests


class TakePhoto:
    # Señal para actualizar el estado de la conexión en la interfaz principal
    def __init__(self, base_url, save_folder="imagenes"):
        self.base_url = base_url
        self.save_folder = save_folder
        # Crear la carpeta si no existe
        os.makedirs(save_folder, exist_ok=True)

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
