import time
import serial
import codecs
from math import *
import numpy as np
import socket
import os
from PyQt5.QtCore import QThread

IP_ADDR = "192.168.0.10"  # NUC
PORT = 8600
roll = pitch = yaw = 0
exit_flag = False  # Se requiere para el bucle de ejecución
_FLAG_CONECT = False  # Se utiliza para controlar la conexión

class IMU(QThread):
    global roll, pitch, yaw, _FLAG_CONECT

    def __init__(self, parent=None):
        super().__init__(None)
        self.falseParent = parent

    def read_file(self, filename):
        try:
            with open(filename, 'r') as file:
                data = file.readline().strip()  # Leer una línea del archivo y eliminar espacios en blanco
                return data.split(', ')  # Dividir la cadena en una lista usando ', ' como separador
        except FileNotFoundError:
            print(f"El archivo {filename} no se encontró.")
            return None
    def read_file2(self, filename):
        try:
            with open(filename, 'w') as file:
                data = file.readline().strip()  # Leer una línea del archivo y eliminar espacios en blanco
                return data.split(', ')  # Dividir la cadena en una lista usando ', ' como separador
        except FileNotFoundError:
            print(f"El archivo {filename} no se encontró.")
            return None

    def run(self):
        # filename_ins = "/home/tumi/imu_int/ins1.txt"
        filename_ins = "IMU/ins1.txt"
        filename_ins2 = "IMU/ins2.txt"

        try:
            while not exit_flag:  # Usar la bandera de salida para controlar el bucle de ejecución
                ins_data = self.read_file(filename_ins)
                if ins_data:
                    pass
                ins_data2 = self.read_file2(filename_ins2)
                if ins_data2:
                    pass
        except Exception as e:
            print(f"El error general es: {e}")

# Crear una instancia de la clase IMU y comenzar su hilo de ejecución
imu_thread = IMU()
imu_thread.start()

# Aquí puedes colocar cualquier código adicional que desees ejecutar en paralelo con el hilo IMU
# Por ejemplo, un bucle infinito para mantener el programa en ejecución
while True:
    # Coloca aquí tu código adicional
    time.sleep(1)  # Asegura que el bucle no consuma demasiados recursos
