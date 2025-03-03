import cv2
import os
import numpy as np
import requests
import logging
import time

logger = logging.getLogger(__name__)


class TakePhoto:
    # Señal para actualizar el estado de la conexión en la interfaz principal
    def __init__(self, base_url, save_folder="imagenes"):
        self.base_url = base_url
        self.save_folder = save_folder
        # Crear la carpeta si no existe
        os.makedirs(save_folder, exist_ok=True)

    def download_photo(self, endpoint, filename="foto.jpg"):
        """Envía una solicitud GET al servidor y guarda la imagen recibida."""
        start = time.perf_counter_ns()

        url = f"{self.base_url}{endpoint}"

        try:
            response = requests.get(url)  # Cambiado a GET
            response.raise_for_status()

            # Convertir los bytes de la respuesta en una imagen
            img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            # Guardar la imagen en la carpeta
            file_path = os.path.join(self.save_folder, filename)
            cv2.imwrite(file_path, frame)

            stop = time.perf_counter_ns()
            logging.info(f"Foto guardado en {file_path} en {(stop-start)/10e6} ms")  # noqa

        except requests.exceptions.RequestException as e:
            logging.exception(f"Error al descargar el frame: {e}")
