import sys
import os
import time
import pygame
from PyQt5 import QtWidgets
import logging
import socket
import signal
import threading
from http.server import HTTPServer
pygame.init()
pygame.joystick.init()
import subprocess

from screens.LoginUi import LoginUi
import utils.RequestHandler as RequestHandler

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *


""" DATOS DE LA CÁMARA """
CAMS_PORT_LIST = [8080, 8085]
FRAME_RATE = 10
WIDTH_FRAME_CAM = 640
HEIGHT_FRAME_CAM = 480


""" LOGGIN FORMAT """
# Creamos el formato del log
FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
logging.basicConfig(level=logging.INFO, format=FORMAT)

""" VARIABLES DEL SISTEMA """
# My system enviroment
MY_FILEPATH = os.path.dirname(os.path.abspath(__file__))




timeout_seconds = 3
socket.setdefaulttimeout(timeout_seconds)

_FLAG_CONECT = False

FLAG_MAPA= False
FLAG_AUTO= False
is_taking_pictures = False  # determina si empezo el proceso de captura de imagenes
# guarda la direccion mas reciente en la que se grabaron imagenes
picture_directory = None
picture_timer = None  # guarda el temporizador para la captura de imagenes
ini = True  # Bandera que indica cuando se empieze a grabar
is_saving= False
id_gen = 1
id_sec = 2


# Threads para conexión WiFi
exit_flag = threading.Event()
is_runing = True
# Funcion que se activa cuando se presiona CTRL+C


def sigint_handler(signum, frame):
    global is_runing
    logging.info("Received SIGINT. Exiting gracefully...")
    is_runing = False
    exit_flag.set()

def JSONRec(server_class=HTTPServer, handler_class=RequestHandler, port=8082):
    try:
        server_address = ('192.168.0.100', port)
        httpd = server_class(server_address, handler_class)
        print(f'Servidor iniciado en el puerto {port} ...')
        httpd.serve_forever()
    except Exception as e:
        logging.error(f'Error en JSONRec: {e}')

def Angle():
    global _FLAG_CONECT, YAW, ROLL, PITCH, SPEED
    while True:
        try:
            if not _FLAG_CONECT:

                command='winscp.com /command "open sftp://tumi:tumi@10.100.108.157" "get data.txt" "exit"'
                subprocess.run(command, shell=True, check=True)

                try:
                    with open("data.txt", 'r') as file:
                        data=file.readline().strip()
                        data= data.split(', ')
                        if data:
                            try:
                                roll, pitch, yaw, speed= [float(value.split(': ')[1]) for value in data[:4]]                        
                                roll=round(roll,0)
                                pitch=round(pitch,0)
                                yaw=round(yaw,0)
                                speed=round(speed,0)
                                message=f"{roll},{pitch},{yaw},{speed}"
                                print("aaaaaaaaa"+message)
                                YAW=yaw
                                ROLL=roll
                                PITCH=pitch
                                SPEED=speed
                            except:
                                pass
                except FileNotFoundError:
                    print(f"No se encontro")
    
        except IndexError:
            print("No llegó un dato anguloso")

def waitForLoading():
    global _FLAG_continue,mesh_item
    t_start=time.time()
    """Creamos el directorio para videos """
    temp_videosFilePath=os.path.join(MY_FILEPATH,"videos")
    if not os.path.exists(temp_videosFilePath):
        ## Creamos el directorio
        os.mkdir(temp_videosFilePath)
        logging.info("Se ha creado el directorio: {}".format(temp_videosFilePath))
    else:
        logging.info("Ya existe el directorio: {}".format(temp_videosFilePath))
    
    """ Verificamos que todo haya ocurrido correctamente """
    while time.time()-t_start<2.5:
        pass
    _FLAG_continue=True

# Se invoca al main desde esta linea
if __name__ == "__main__":
    
    app = QtWidgets.QApplication(sys.argv)
    signal.signal(signal.SIGINT, sigint_handler)
    threading.Thread(target=JSONRec, daemon=True).start()
    threading.Thread(target=Angle,daemon=True).start()


    import pics_rc
    splash_image = QPixmap("iconos/loadScreen.png")
    splash = QSplashScreen(splash_image)
    splash.show()
    _FLAG_continue=False
    th=threading.Thread(target=waitForLoading,)
    th.start()
    while not(_FLAG_continue): pass
    splash.close()

    mi_app = LoginUi()
    mi_app.show()
    sys.exit(app.exec_())
