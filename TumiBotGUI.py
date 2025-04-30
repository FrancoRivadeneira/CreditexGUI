
from utils.CajaSensoresNetworkCheckThread import CajaSensoresNetworkCheckThread
from utils.CajaSensorReader import CajaSensorReader
from utils.Camera import Camera
from utils.cppjsonReception import cppjsonReception
from utils.customDialog import CustomDialog
from utils.EncoderDataReader import EncoderDataReader
from utils.GrabarReader import GrabarReader
from utils.imuDataReader import imuDataReader
from utils.InfoReader import InfoReader
from utils.JoystickControl import JoystickControl
from utils.Mid360DataReader import Mid360DataReader
from utils.RobotNetworkCheckThread import RobotNetworkCheckThread
from utils.RobotReader import RobotReader
from utils.socketReception import socketReception
# from utils.StopReader import StopReader
from utils.HoraUpdater import HoraUpdater
from utils.DateUpdater import DateUpdater
from utils.TakePhoto import TakePhoto
from utils.PTZ import PTZ
from utils.FullScreenImage import FullScreenImage
from MainWindow import Ui_MainWindow

# Library for PyQt
import PyQt5.QtCore as QtCore
from PyQt5.QtWidgets import QMainWindow, QApplication, QGridLayout, QLabel, QPushButton, QFrame, QDialog, QVBoxLayout, QScrollArea, QWidget, QSplashScreen
from PyQt5.QtCore import QTimer, pyqtSlot
from PyQt5.QtGui import QMatrix4x4, QPixmap, QIcon, QFont, QMovie, QImage
from PyQt5.QtCore import Qt, QSize  # Agregar QSize a la importación

import cv2  # For Camera visualization  ###
import sys
import os
import logging  # Libreria para info Logging
import numpy as np
import pandas as pd
import time
import threading  # For hilos
# from pygame.locals import *
import pickle
# import paramiko
# import pyqtgraph.opengl as gl  ###
# import pyqtgraph as pg
import datetime

FLAG_AUTO = False
FLAG_MAPA = False
_FLAG_CONECT = True
""" COMANDOS PARA LA NUCLEO"""

comandoList = [
    ["$OAX3J0A", [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], "Avanzar"],  # noqa
    ["$OAX3J0B", [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], "Retroceder"],  # noqa
    ["$OAX3J0r", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], "Giro Horario"],  # noqa
    ["$OAX3J0l", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], "Giro Antihorario"],  # noqa
    ["$OAX3JR1", [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], "Prender Luces"],  # noqa
    ["$OAX3JR2", [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], "Apagar Luces"],  # noqa
    ["$OAX3JX1", [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0], "Aumentar Velocidad"],  # noqa
    ["$OAX3JX2", [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], "Disminuir Velocidad"],  # noqa
    ["$OAX3JRA", [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], "Avanzar Derecha"],  # noqa
    ["$OAX3JLA", [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], "Avanzar Izquierda"],  # noqa
    ["$OAX1M", [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0], "Activar Motores"],  # CAMBIAR ESTO es Y + FlechaArriba # noqa
    ["$OAX3JAR", [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], "Apagar Rele"],  # CAMBIAR ESTO es Y + FlechaAbajo # noqa
    ["$OAX3JBE", [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], "Boton Emergencia"],  # CAMBIAR ESTO es Y + FlechaIzquierda # noqa
]

# Aplicamos transpuesta a la lista de comandos para hacer uso del built function "index"

""" LOGGIN FORMAT """
# Creamos el formato del log
FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
logging.basicConfig(level=logging.INFO, format=FORMAT)

# My system enviroment
MY_FILEPATH = os.path.dirname(os.path.abspath(__file__))

# joystick
EVENT_TYPE_LEFT_TRIGGER = 4
if sys.platform == "linux":
    EVENT_TYPE_LEFT_TRIGGER = 2

# ROBOT INFO (RASPBERRY REMOTE)
# ROBOT_TERRESTRE_IP_ADDR=  "192.168.1.166" #"192.168.1.166""192.168.1.133"
ROBOT_TERRESTRE_IP_ADDR = "10.100.108.157"  # "192.168.1.166""192.168.1.133"
ROBOT_TERRESTRE_USERNAME = "pi"
ROBOT_TERRESTRE_PASSWORD = "raspberry"
ROBOT_TERRESTRE_SSH_PORT = 22  # Puerto por defecto de SSH
ROBOT_TERRESTRE_PORT = 8666
ROBOT_TERRESTRE_CAMS_PORT_LIST = [8080, 8085, 8088]
ROBOT_TERRESTRE_FRAME_RATE = 10
ROBOT_TERRESTRE_WIDTH_FRAME_CAM = 640
ROBOT_TERRESTRE_HEIGHT_FRAME_CAM = 480
ROBOT_TERRESTRE_MIN_VALOR_BATERIA = 22
ROBOT_TERRESTRE_RANGO_BATERIA = 25-22

# LIDAR Module Info
LIDAR_MODULE_IP_ADDR = "192.168.1.4"  # "192.168.137.4"
LIDAR_MODULE_USERNAME = "jetson"  # "pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  # "raspberry" #"nvidia"


class GUI(QMainWindow):
    def __init__(self):
        # global mesh_item
        QMainWindow.__init__(self)
        """ Configuramos GUI"""
        # Creamos nuestra GUI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.map_df = pd.read_csv('maps.csv')
        self.initialize_map_frames()

        """ DEFINICION de FLAG """
        self._FLAG_buttonOnPressed = False
        self._FLAG_socketConected = False
        self._FLAG_sshImuConnected = False
        self._FLAG_sshLidarConnected = False
        self._FLAG_sshMID360Connected = False

        self._sshRobotConnected = False
        self._sshCajaSensoresConnected = False
        self._sshGrabacionConnected = False
        self._sshStopConnected = False

        self._FLAG_LidarFuncionando = False

        self._FLAG_MID360Funcionando = False

        self._FLAG_sshEncoderConnected = False
        self._FLAG_EncoderFuncionando = False

        self._FLAG_SOCKET_ROBOT_CONNECTED = False

        self._FLAG_Habilitar_Robot = 0
        self._FLAG_Habilitar_Sensores = 0
        self._FLAG_SENSORES_pressed = False
        self._FLAG_Habilitar_Grabacion = 0
        self._Flag_BotonGrabar_pressed = False
        self._FLAG_Habilitar_Continuar = 0

        """ DEFINICION de Variables """
        # Variables dependientes del tipo del Robot
        self._VAR_USERNAME = ROBOT_TERRESTRE_USERNAME
        self._VAR_PASSWORD = ROBOT_TERRESTRE_PASSWORD
        self._VAR_IP_ADDR = ROBOT_TERRESTRE_IP_ADDR
        self._VAR_PORT = ROBOT_TERRESTRE_PORT
        self._VAR_CAMS_PORT_LIST = ROBOT_TERRESTRE_CAMS_PORT_LIST
        self._VAR_FRAME_RATE = ROBOT_TERRESTRE_FRAME_RATE
        self._VAR_WIDTH_FRAME_CAM = ROBOT_TERRESTRE_WIDTH_FRAME_CAM
        self._VAR_HEIGHT_FRAME_CAM = ROBOT_TERRESTRE_HEIGHT_FRAME_CAM
        self._VAR_MIN_VALOR_BATERIA = ROBOT_TERRESTRE_MIN_VALOR_BATERIA
        self._VAR_RANGO_BATERIA = ROBOT_TERRESTRE_RANGO_BATERIA

        self._VAR_Nombre_GRabacion = ""
        # socket
        self._VAR_socketClient = None

        self._VAR_socketClienteROBOT = None
        # Video counter
        self._VAR_counterVideoFiles = 0
        # List that contains the VideoWriter to MP4 of the video
        self._VAR_outVideoWrite_list = [None, None]
        self._VAR_tramaDatos = ""
        # Objeto Mesh
        # self._VAR_meshRobot=mesh_item
        # Objetos detector internet:
        self._Var_valueCajaSensores = 0
        self._Var_valueRobot = 0

        pixmap = QPixmap("iconos/Toggle.svg")
        pixmap = pixmap.scaled(60, 60,
                               QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                               QtCore.Qt.TransformationMode.SmoothTransformation)
        icon = QIcon(pixmap)
        self.ui.btn_toggle.setIcon(icon)
        self.ui.btn_toggle.setIconSize(QtCore.QSize(60, 60))
        del pixmap, icon

        """  Realizamos el conteo de videos en la computadora """
        temp_videosFilePath = os.path.join(MY_FILEPATH, "videos")
        self._VAR_counterVideoFiles = len(os.listdir(temp_videosFilePath))

        """ Cargamos los visualizadores"""
        # Esto es de cuando el lidar se visualizaba en 2D
        # grid_item = gl.GLGridItem()
        # self.ui.visLidar.addItem(grid_item)

        # #Esto es de cuando el lidar se visualizaba en 3D
        # self.scatter = gl.GLScatterPlotItem(
        #     pos=np.zeros((1, 3)),  # Inicialmente vacío
        #     size=2,
        #     color=(1, 1, 1, 1),  # Color blanco
        #     pxMode=True)
        # self.ui.visLidar.addItem(self.scatter)

        # self.ui.visImu.addItem(self._VAR_meshRobot)
        # self.ui.visImu.addItem(grid_item)

        # >>> Visualizador Roll
        # Definimos limite
        # self.ui.visRoll.setXRange(-1, 1)
        # self.ui.visRoll.setYRange(-1, 1)
        # # Quitar las marcas de los ejes X e Y (sin números en los costados)
        # self.ui.visRoll.getPlotItem().getAxis('bottom').setTicks([])  # Quita las marcas del eje X
        # self.ui.visRoll.getPlotItem().getAxis('left').setTicks([])    # Quita las marcas del eje Y
        # # (Opcional) Si también deseas quitar las líneas de los ejes X e Y:
        # self.ui.visRoll.getPlotItem().getAxis('bottom').setPen(None)  # Quita la línea del eje X
        # self.ui.visRoll.getPlotItem().getAxis('left').setPen(None)    # Quita la línea del eje Y
        # # Definimos las lineas a plotear
        # self._VAR_rollReference_LineBase = self.ui.visRoll.plot(pen='r')
        # self._VAR_rollReference_Line = self.ui.visRoll.plot(pen='g')
        # self._VAR_rollReference_Square = self.ui.visRoll.plot(pen='y')
        # # Definimos los puntos del cuadrado
        # self._VAR_rollSquarePoints = np.array([[-0.2, -0.2], [0.2, -0.2], [0.2, 0.3], [0.1,0.3], [0.1,0.2], [-0.1, 0.2], [-0.1, 0.3], [-0.2, 0.3] ,[-0.2, -0.2]])
        # # Definimos los puntos de las lineas base
        # angle=0
        # x1 = 1 * np.cos(np.radians(angle))
        # y1 = 1 * np.sin(np.radians(angle))
        # x2 = -1 * np.cos(np.radians(angle))
        # y2 = -1 * np.sin(np.radians(angle))
        # self._VAR_rollLinePoints=np.array([[x1,y1],[x2,y2]])
        # # Establecemos la data al plot
        # self._VAR_rollReference_Square.setData(self._VAR_rollSquarePoints[:, 0], -self._VAR_rollSquarePoints[:, 1])
        # self._VAR_rollReference_Line.setData(self._VAR_rollLinePoints[:, 0], self._VAR_rollLinePoints[:, 1])
        # self._VAR_rollReference_LineBase.setData(self._VAR_rollLinePoints[:, 0], self._VAR_rollLinePoints[:, 1])

        # ## >>> Visualizador Pitch
        # # Definimos limite
        # self.ui.visPitch.setXRange(-1, 1)
        # self.ui.visPitch.setYRange(-1, 1)
        # self.ui.visPitch.getPlotItem().getAxis('bottom').setTicks([])  # Quita las marcas del eje X
        # self.ui.visPitch.getPlotItem().getAxis('left').setTicks([])    # Quita las marcas del eje Y
        # # (Opcional) Si también deseas quitar las líneas de los ejes X e Y:
        # self.ui.visPitch.getPlotItem().getAxis('bottom').setPen(None)  # Quita la línea del eje X
        # self.ui.visPitch.getPlotItem().getAxis('left').setPen(None)    # Quita la línea del eje Y
        # # Definimos las lineas a plotear
        # self._VAR_pitchReference_LineBase = self.ui.visPitch.plot(pen='r')
        # self._VAR_pitchReference_Line = self.ui.visPitch.plot(pen='g')
        # self._VAR_pitchReference_Square = self.ui.visPitch.plot(pen='y')
        # # Definimos los puntos del cuadrado
        # self._VAR_pitchSquarePoints = np.array([[-0.7, -0.2], [0.7, -0.2], [0.7, 0.2], [0.6, 0.2] , [0.6, 0.3], [0.45, 0.3], [0.45,0.2], [-0.45, 0.2], [-0.45, 0.3],
        # [-0.6,0.3], [-0.6,0.2], [-0.7 , 0.2],  [-0.8, 0.0],[-0.7, -0.2]])
        # # Definimos los puntos de las lineas base
        # angle=0
        # x1 = 1 * np.cos(np.radians(angle))
        # y1 = 1 * np.sin(np.radians(angle))
        # x2 = -1 * np.cos(np.radians(angle))
        # y2 = -1 * np.sin(np.radians(angle))
        # self._VAR_pitchLinePoints=np.array([[x1,y1],[x2,y2]])
        # # Establecemos la data al plot
        # self._VAR_pitchReference_Square.setData(self._VAR_pitchSquarePoints[:, 0], -self._VAR_pitchSquarePoints[:, 1])
        # self._VAR_pitchReference_Line.setData(self._VAR_pitchLinePoints[:, 0], self._VAR_pitchLinePoints[:, 1])
        # self._VAR_pitchReference_LineBase.setData(self._VAR_pitchLinePoints[:, 0], self._VAR_pitchLinePoints[:, 1])

        # ## >>> Visualizador Yaw
        # # Definimos limite
        # self.ui.visYaw.setXRange(-1, 1)
        # self.ui.visYaw.setYRange(-1, 1)
        # self.ui.visYaw.getPlotItem().getAxis('bottom').setTicks([])  # Quita las marcas del eje X
        # self.ui.visYaw.getPlotItem().getAxis('left').setTicks([])    # Quita las marcas del eje Y
        # # (Opcional) Si también deseas quitar las líneas de los ejes X e Y:
        # self.ui.visYaw.getPlotItem().getAxis('bottom').setPen(None)  # Quita la línea del eje X
        # self.ui.visYaw.getPlotItem().getAxis('left').setPen(None)    # Quita la línea del eje Y
        # # Definimos las lineas a plotear
        # self._VAR_yawReference_LineBase = self.ui.visYaw.plot(pen='r')
        # self._VAR_yawReference_Line = self.ui.visYaw.plot(pen='g')
        # self._VAR_yawReference_Square = self.ui.visYaw.plot(pen='y')
        # # Definimos los puntos del cuadrado
        # self._VAR_yawSquarePoints = np.array([[-0.7, -0.2], [-0.6, -0.2], [-0.6, -0.3], [-0.45, -0.3], [-0.45, -0.2], [0.45, -0.2], [0.45, -0.3], [0.6 , -0.3], [0.6, -0.2],
        # [0.7, -0.2], [0.7, 0.2],[0.6,0.2],[0.6, 0.3], [0.45, 0.3], [0.45,0.2],[-0.45,0.2],[-0.45,0.3], [-0.6,0.3], [-0.6,0.2], [-0.7, 0.2],[-0.8, 0.0], [-0.7, -0.2]])
        # # Definimos los puntos de las lineas base
        # angle=0
        # x1 = 1 * np.cos(np.radians(angle))
        # y1 = 1 * np.sin(np.radians(angle))
        # x2 = -1 * np.cos(np.radians(angle))
        # y2 = -1 * np.sin(np.radians(angle))
        # self._VAR_yawLinePoints=np.array([[x1,y1],[x2,y2]])
        # # Establecemos la data al plot
        # self._VAR_yawReference_Square.setData(self._VAR_yawSquarePoints[:, 0], -self._VAR_yawSquarePoints[:, 1])
        # self._VAR_yawReference_Line.setData(self._VAR_yawLinePoints[:, 0], self._VAR_yawLinePoints[:, 1])
        # self._VAR_yawReference_LineBase.setData([1,-1], [0,0])

        """ DEFINICION de los hilos"""
        # verificacion de red de internet
        # verificacion de la red de internet del robot
        self._THREAD_RobotNetwork_thread = RobotNetworkCheckThread(self)
        self._THREAD_RobotNetwork_thread.connection_status_signal.connect(self.ui.visImu.append)  # noqa
        self._THREAD_RobotNetwork_thread.connection_status.connect(self.Robot_update_connection_status)  # noqa
        # verificacion de la red de internet de la caja de sensores
        self._THREAD_CajaSensoresNetwork_thread = CajaSensoresNetworkCheckThread(self)  # noqa
        self._THREAD_CajaSensoresNetwork_thread.connection_status_signal.connect(self.ui.visImu.append)  # noqa
        self._THREAD_CajaSensoresNetwork_thread.connection_status.connect(self.Sensor_update_connection_status)  # noqa

        # Crea el socket de comunicacion y recepcion de datos con el robot
        self._THREAD_socketReceptionData = socketReception(self)
        # self._THREAD_socketReceptionData._SIGNAL_msg.connect(self.ui.visImu.append)
        self._THREAD_socketReceptionData._SIGNAL_tramaDatos.connect(self.updateIndicators)  # noqa
        # Check the joystick connection status and update command
        self._THREAD_joytickCommand = JoystickControl(self)
        self._THREAD_joytickCommand._SIGNAL_command.connect(self.sendJoystickCommand)  # noqa
        # self._THREAD_joytickCommand._SIGNAL_updateJoystickIndicator.connect(self.ui.progressBar_mando.setValue)
        # Camaras
        self._THREAD_updateCamera1 = Camera(self, 1)
        self._THREAD_updateCamera1.change_pixmap_signal.connect(self.update_image2)  # noqa

        self._THREAD_updateCamera2 = Camera(self, 2)
        self._THREAD_updateCamera2.change_pixmap_signal.connect(self.update_image3)  # noqa

        self._THREAD_updateCamera3 = PTZ(self)
        self._THREAD_updateCamera3.change_pixmap_signal.connect(self.update_image1)  # noqa
        # self._THREAD_updateCamera3.change_pixmap_signal.connect(self.update_image3)
        # SSH Imu
        self._THREAD_imuDataReader = imuDataReader(self)
        self._THREAD_imuDataReader._SIGNAL_data.connect(self.updateMesh)

        # JSON Lidar
        # self._THREAD_JSONMID360DataReader = jsonReception(self)
        # self._THREAD_JSONMID360DataReader._SIGNAL_data.connect(self.updateLidar3D)

        # Json Lidar cpp

        self._THREAD_CPPJSONMID360 = cppjsonReception(self)
        self._THREAD_CPPJSONMID360._SIGNAL_data.connect(self.updateLidar3D)
        # self._THREAD_CPPJSONMID360.start()

        # SSH Lidar  se solia usar para el ydlidar su visualizador solia ser visLidar
        # self._THREAD_imuLidarReader=lidarDataReader(self)
        # self._THREAD_imuLidarReader._SIGNAL_data.connect(self.updateLidar)

        # SSH MID360
        self._THREAD_MID360Reader = Mid360DataReader(self)
        self._THREAD_MID360Reader._SIGNAL_data.connect(self.updateMID360)

        # SSH Encoder
        self._THREAD_EncoderLidarReader = EncoderDataReader(self)
        self._THREAD_EncoderLidarReader._SIGNAL_data.connect(self.updateEncoder)  # noqa

        # Hora Updater
        self._THREAD_HoraUpdater = HoraUpdater(self)
        self._THREAD_HoraUpdater._SIGNAL_data.connect(self.updateHour)
        self._THREAD_HoraUpdater.start()

        # Date Updater

        self._THREAD_DateUpdater = DateUpdater(self)
        self._THREAD_DateUpdater._SIGNAL_data.connect(self.updateDate)
        self._THREAD_DateUpdater.start()

        """ Definicion de Timers"""
        # Robot Imu
        self._TIMER_updateMesh = QTimer()
        self._TIMER_updateMesh.timeout.connect(self.updateMesh)

        # Robot Lidar
        # self._TIMER_updateLidar=QTimer()
        # self._TIMER_updateLidar.timeout.connect(self.updateLidar)

        """  Conectamos los eventos de los objetos con las funciones """
        self.ui.Encender_button.pressed.connect(self.BUTTON_ON_pressed)
        # self.ui.Apagar_button.pressed.connect(self.BUTTON_OFF_pressed)
        # self.ui.Sensores_button.pressed.connect(self.BUTTON_SENSORS_pressed)
        self.ui.button_Grabar.pressed.connect(self.BUTTON_GRABAR_pressed)
        self.ui.btn_photo.pressed.connect(self.TAKE_PHOTO)
        self.ui.button_stop.pressed.connect(self.BUTTON_STOP_pressed)
        self.ui.button_info.pressed.connect(self.BUTTON_INFO_pressed)
        self.ui.close_window_button.clicked.connect(lambda: self.close())
        self.ui.btn_toggle.clicked.connect(self.TOGGLE_MODE)
        self.ui.btn_test.clicked.connect(self.depuracion)

        self.ui.btn_ptz_down.pressed.connect(self._THREAD_updateCamera3.move_down)  # noqa
        self.ui.btn_ptz_down.released.connect(self._THREAD_updateCamera3.stop_move)  # noqa

        self.ui.btn_ptz_right.pressed.connect(self._THREAD_updateCamera3.move_right)  # noqa
        self.ui.btn_ptz_right.released.connect(self._THREAD_updateCamera3.stop_move)  # noqa

        self.ui.btn_ptz_left.pressed.connect(self._THREAD_updateCamera3.move_left)  # noqa
        self.ui.btn_ptz_left.released.connect(self._THREAD_updateCamera3.stop_move)  # noqa

        self.ui.btn_ptz_up.pressed.connect(self._THREAD_updateCamera3.move_up)  # noqa
        self.ui.btn_ptz_up.released.connect(self._THREAD_updateCamera3.stop_move)  # noqa

        self.ui.btn_zoom_in.pressed.connect(self._THREAD_updateCamera3.zoom_in)  # noqa
        self.ui.btn_zoom_in.released.connect(self._THREAD_updateCamera3.stop_move)  # noqa

        self.ui.btn_zoom_out.pressed.connect(self._THREAD_updateCamera3.zoom_out)  # noqa
        self.ui.btn_zoom_out.released.connect(self._THREAD_updateCamera3.stop_move)  # noqa

        """ Modos la GUI """
        self.ui.btn_general.clicked.connect(self.distribuir_general)
        # self.ui.btn_parada.clicked.connect(self.parada)
        self.ui.btn_angulo.clicked.connect(self.distribuir_arm)
        self.ui.btn_document.clicked.connect(self.distribuir_document)
        self.ui.btn_archivos.clicked.connect(self.distribuir_archivos)
        self.ui.btn_posicion.clicked.connect(self.distribuir_posicion)
        self.ui.btn_tumi.clicked.connect(self.distribuir_secret)
        # self.ui.btn_reset.clicked.connect(self.reset_button)
        self.ui.btn_estadisticas.clicked.connect(self.GoEstadisticas)
        self.ui.btn_generarReport.clicked.connect(self.GoGenerarReport)

        """ Iniciamos hilos o timers """
        self._THREAD_RobotNetwork_thread.start()
        self._THREAD_CajaSensoresNetwork_thread.start()
        # self._THREAD_socketReceptionData.start()

        # Chequea si el joystick está o no conectado y envia la trama al robot
        self._THREAD_joytickCommand.start()

    def Robot_update_connection_status(self, status):
        # Agregar el nuevo estado al final del QTextBrowser
        if status == 1:
            self._FLAG_Habilitar_Robot = 1

        else:
            self._FLAG_Habilitar_Robot = 0

    def Sensor_update_connection_status(self, status):
        # Agregar el nuevo estado al final del QTextBrowser
        if status == 1:
            self._FLAG_Habilitar_Sensores = 1

        else:
            self._FLAG_Habilitar_Sensores = 0
            self._FLAG_Habilitar_Continuar = 0
            self.ui.button_Grabar.setStyleSheet(
                """QPushButton {
                font-weight: bold;  /* Siempre en negrita */
                }
                """)

    def updateMesh(self, data):
        # Definimo la matriz de transformación
        transform = QMatrix4x4()
        # print(data)
        """ Roll """
        angle = data[0]+180
        # >> vis IMU
        # Realizamos la rotacion al Robot Mesh
        transform.rotate(angle, 1, 0, 0)
        # >> vis Roll
        # Matrix de rotacion
        rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                                    [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])
        # Rotar los puntos del cuadrado
        rollSquarePoints = np.dot(self._VAR_rollSquarePoints, rotation_matrix)
        # Rotar los puntos de la línea
        rollLinePoints = np.dot(self._VAR_rollLinePoints, rotation_matrix)
        # Actualizar los datos en los trazados
        self._VAR_rollReference_Square.setData(
            rollSquarePoints[:, 0], rollSquarePoints[:, 1])
        self._VAR_rollReference_Line.setData(
            rollLinePoints[:, 0], rollLinePoints[:, 1])
        self.ui.label_visRoll.setText(
            f'Roll: <span style="color: #90EE90;">{angle-180:.1f}°</span>')
        """ Pitch """
        angle = data[1]+180
        # >> vis IMU
        # Realizamos la rotacion al Robot Mesh
        transform.rotate(angle, 0, 1, 0)
        # >> vis Pitch
        # Matrix de rotacion
        rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                                    [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])
        # Rotar los puntos del cuadrado
        pitchSquarePoints = np.dot(
            self._VAR_pitchSquarePoints, rotation_matrix)
        # Rotar los puntos de la línea
        pitchLinePoints = np.dot(self._VAR_pitchLinePoints, rotation_matrix)
        # Actualizar los datos en los trazados
        self._VAR_pitchReference_Square.setData(
            pitchSquarePoints[:, 0], pitchSquarePoints[:, 1])
        self._VAR_pitchReference_Line.setData(
            pitchLinePoints[:, 0], pitchLinePoints[:, 1])
        self.ui.label_visPitch.setText(
            f'Pitch: <span style="color: #90EE90;">{angle-180:.1f}°</span>')
        """ Yaw """
        angle = data[2]+180
        # >> vis IMU
        # Realizamos la rotacion al Robot Mesh
        transform.rotate(angle, 0, 0, 1)
        # >> vis Yaw
        # Matrix de rotacion
        rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                                    [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])
        # Rotar los puntos del cuadrado
        yawSquarePoints = np.dot(self._VAR_yawSquarePoints, rotation_matrix)
        # Rotar los puntos de la línea
        yawLinePoints = np.dot(self._VAR_yawLinePoints, rotation_matrix)
        # Actualizar los datos en los trazados
        self._VAR_yawReference_Square.setData(
            yawSquarePoints[:, 0], yawSquarePoints[:, 1])
        self._VAR_yawReference_Line.setData(
            yawLinePoints[:, 0], yawLinePoints[:, 1])
        self.ui.label_visImuYaw.setText(
            f'Yaw: <span style="color: #90EE90;">{angle-180:.1f}°</span>')

        """ Actualizamos la transformacion del Mesh"""
        # Aplicar la nueva matriz de transformación al objeto
        # self._VAR_meshRobot.setTransform(transform)

    # def updateLidar(self, data):
    #     # print("a")
    #     tempRanges = eval(data)
    #     # print(tempRanges)
    #     temp_numPoints = len(tempRanges)
    #     angle_min = -3.1415927410125732
    #     angle_max = 3.1415927410125732
    #     # Se puede sumar un valor de angulo si el lidar estaba mal colocado
    #     angles = np.linspace(angle_min, angle_max, temp_numPoints)
    #     # Obtenemos los valores de XY del anillo del Lidar
    #     tempX = tempRanges*np.cos(angles)
    #     tempY = tempRanges*np.sin(angles)
    #     pointCloud = (np.vstack((tempX, -tempY))).T
    #     # Ploteamos en el Vis
    #     item = pg.ScatterPlotItem(pxMode=True, pos=pointCloud, size=2)
    #     self.ui.visLidar.clear()
    #     self.ui.visLidar.addItem(item)
    #     if self._FLAG_LidarFuncionando == True:
    #         # self.ui.progressBar_lidar.setValue(100)
    #         pass
    #     else:
    #         # self.ui.progressBar_lidar.setValue(0)
    #         pass
    #     if data == None:
    #         # self.ui.progressBar_lidar.setValue(0)
    #         pass

    def updateLidar3D(self, data):
        self.scatter.setData(pos=data, size=0.1, color=(1, 1, 1, 1))

    def updateMID360(self, data):

        if self._FLAG_MID360Funcionando == True:
            # self.ui.progressBar_lidar.setValue(100)
            pass
        else:
            # self.ui.progressBar_lidar.setValue(0)
            pass

        if data == None:
            # self.ui.progressBar_lidar.setValue(0)
            pass

    def updateHour(self, data):
        data = str(data)
        self.ui.horaLabel.setText(data)

    def updateDate(self, data):
        data = str(data)
        self.ui.dateLabel.setText(data)

    def updateEncoder(self, data):
        Velocidades = eval(data)
        texto_para_label = str(Velocidades)
        # Self para mostrar la data de la velocidad
        self.ui.encoder_data.setText(texto_para_label)
        # Verificar el contenido del mensaje y aplicar el color correspondiente
        if texto_para_label == "[0, 0, 0, 0, 0, 0, 0, 0]":
            self.ui.encoder_data.setStyleSheet(
                "background-color: white;color: red")
        else:
            self.ui.encoder_data.setStyleSheet(
                "background-color: white;color: green")

        if self._FLAG_EncoderFuncionando == True:
            self.ui.progressBar_encoder.setValue(100)
        else:
            self.ui.progressBar_encoder.setValue(0)

        if data == None:
            self.ui.progressBar_encoder.setValue(0)

    def updateIndicators(self, tramaDatos: str):
        if tramaDatos == "" or not (self._FLAG_socketConected):
            return
        try:
            logging.info("Trama recibida: {}".format(tramaDatos[:-1]))
            comprobar = tramaDatos[0:6]
            tramaSensores = tramaDatos[6:]
            if comprobar == '$OAX1s':  # $OAX1jb145s15l0m11r0d1p1
                logging.debug(f"Comprobar es {tramaSensores}")
                i_bat = tramaSensores.find('b')
                i_speed = tramaSensores.find('v')
                i_lights = tramaSensores.find('l')
                i_motors = tramaSensores.find('m')
                i_lidar = tramaSensores.find('n')
                i_dust = tramaSensores.find('d')
                i_path = tramaSensores.find('p')
                i_encoder = tramaSensores.find('e')
                # lidar= tramaDatos.find('r')

                # b:140 170

                if (i_bat != -1) and (i_speed != -1):
                    bat = tramaSensores[(i_bat+1):i_speed]
                    logging.debug(f"Valor de la bateria: {bat}")
                    if bat == 'e':
                        logging.info('Error')
                    else:
                        if bat.isnumeric():
                            valorBateria = int(bat) / 10. / 3.95
                            logging.debug(f"Valor de la bateria: {valorBateria}")  # noqa
                            self.ui.label_bateria.setText(f"Bateria: {valorBateria:.1f} V")  # noqa
                            # normalizamos el valor de la batería
                            porcentaje = (valorBateria - self._VAR_MIN_VALOR_BATERIA) / self._VAR_RANGO_BATERIA * 100  # noqa
                            if porcentaje < 0:
                                self.ui.progressBar_bateria.setValue(0)
                            elif porcentaje > 100:
                                self.ui.progressBar_bateria.setValue(100)
                            else:
                                self.ui.progressBar_bateria.setValue(int(porcentaje))  # noqa
                if (i_speed != -1) and (i_lights != -1):
                    speed = tramaSensores[(i_speed+1):i_lights]
                    if speed.isnumeric():
                        valorVelocidad = int(speed)
                        self.ui.label_velocidad.setText("Velocidad:")
                        self.ui.progressBar_velocidad.setValue(valorVelocidad)
                if (i_lights != -1) and (i_motors != -1):
                    ligths = tramaSensores[(i_lights+1):i_motors]
                    if ligths.isnumeric():
                        valorLights = int(ligths)
                        if valorLights == 1:
                            # self.ui.progressBar_luces.setValue(100)
                            pass

                        if valorLights == 0:
                            # self.ui.progressBar_luces.setValue(0)
                            pass
                if i_motors != -1:
                    motors = tramaSensores[(i_motors+1):i_lidar]  # Hay 4 motores # noqa
                    for id, motor in enumerate(motors):
                        if motor.isnumeric():
                            valorMotor = int(motor)
                            if valorMotor == 0:
                                command = f"self.ui.progressBar_motor{id+1}.setValue(0)"
                            if valorMotor == 1:
                                command = f"self.ui.progressBar_motor{id+1}.setValue(100)"
                            eval(command)
                if i_lidar != -1:
                    numCloud = tramaSensores[i_lidar+1:i_dust]
                    if numCloud.isnumeric():
                        # self.ui.textBrowser_lidar.setText(numCloud)
                        logging.debug("lidar cloud")
                if i_dust != -1:
                    dust = tramaSensores[(i_dust+1):i_path]
                    if dust.isnumeric():
                        valorDust = int(dust)
                        if valorDust == 0:
                            self.ui.progressBar_polvo.setValue(0)
                        if valorDust == 1:
                            self.ui.progressBar_polvo.setValue(1000)
                if i_path != -1:
                    path = tramaSensores[(i_path+1):-i_encoder]
                    if path.isnumeric():
                        valorPath = int(path)
                        if valorPath == 0:
                            self.ui.progressBar_path.setValue(0)
                        if valorPath == 1:
                            self.ui.progressBar_path.setValue(100)
                if i_encoder != -1:
                    valorEncoder = tramaSensores[(i_encoder+1):-1]
                    logging.debug(f"Valor del encoder: {valorEncoder}")
                    # valorEncoder = int(valorEncoder)
                    self.ui.label_altura.setText(f'{valorEncoder} cm')
                    # if valorEncoder.isnumeric():

        except Exception as e:
            logging.info('Datos erroneos! [ERROR:{}]'.format(e))

    def sendJoystickCommand(self, comando):
        try:
            if self._FLAG_socketConected:
                # enviamos el comando a traves del socket
                self._VAR_socketClient.sendall(pickle.dumps(comando))
                if comando == "$OAX3JAR":  # Apagar relé
                    # Esperamos un tiempo adicional para volver a enviar comandos
                    time.sleep(2)
        except:
            logging.info("Error transmitiendo datos por el socket.")
            time.sleep(1)

    def BUTTON_SENSORS_pressed(self):
        if (self._FLAG_Habilitar_Sensores == 1):

            if self._FLAG_SENSORES_pressed:
                self.ui.visImu.append(
                    "<font color='black'>> Boton sensores ya presionado </font>")
                return
            self._FLAG_Habilitar_Grabacion = 1
            self.Thread_CajaSensores = CajaSensorReader(self)
            self.Thread_CajaSensores.start()
            self.ui.visImu.append(
                "<font color='black'>> Los sensores han sido inicializados. Por favor, espere 40 segundos para que el proceso finalice.</font>")

            # Configura un QTimer para iniciar los demás hilos después de 60 segundos
            self.timer = QTimer(self)
            # Asegura que el temporizador se ejecute solo una vez
            self.timer.setSingleShot(True)
            # Conecta al método que inicia los hilos
            self.timer.timeout.connect(self.start_sensor_threads)
            self.timer.start(40000)  # 60000 milisegundos = 60 segundos

            self._FLAG_SENSORES_pressed = True
        else:
            self.ui.visImu.append(
                "<font color='black'>>Espere a una conexión con la Caja Sensores...</font>")
            self._FLAG_Habilitar_Grabacion = 0
            # self._FLAG_SENSORES_pressed = False

    def BUTTON_GRABAR_pressed(self):

        if (self._FLAG_Habilitar_Grabacion == 1):

            dialog = CustomDialog()
            if dialog.exec_() == QDialog.Accepted:
                texto = dialog.line_edit.text().replace(
                    ' ', '_').replace('.', '_').replace(',', '_')
                self._VAR_Nombre_GRabacion = texto

                if self._FLAG_Habilitar_Continuar == 1:
                    self.ui.visImu.append(
                        "<font color='black'>>Ya esta presionado el boton grabacion</font>")
                    return

                self._FLAG_Habilitar_Continuar = 1

                self.ui.button_Grabar.setStyleSheet("""
                    QPushButton {
                        background-color: #e74c3c;  /* Rojo */
                        color: white;
                        font-weight: bold;
                    }""")

                self.Thread_CajaGrabacion = GrabarReader(self)
                self.Thread_CajaGrabacion.connection_status_signal.connect(
                    self.ui.visImu.append)
                self.Thread_CajaGrabacion.start()

            else:
                self.ui.visImu.append(
                    "<font color='black'>>Se cancelo grabacion</font>")

        else:
            self.ui.visImu.append(
                "<font color='black'>>Los sensores todavia no estan emitiendo datos...</font>")

    def TAKE_PHOTO(self):
        logging.info("Inicio de toma de fotos")
        self.TakePhoto = TakePhoto(
            base_url=f"http://{ROBOT_TERRESTRE_IP_ADDR}:1234")
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"Foto_{current_time}.bmp"
        self.TakePhoto.download_photo(endpoint="/ptz_snapshot",
                                      filename=filename)

    def BUTTON_STOP_pressed(self):
        if (self._FLAG_Habilitar_Grabacion == 1):
            self.ui.button_Grabar.setStyleSheet("""QPushButton {
                    font-weight: bold;  /* Siempre en negrita */
                }""")
            # self.Thread_StopGrabacion = StopReader(self)
            # self.Thread_StopGrabacion.connection_status_signal.connect(
            #     self.ui.visImu.append)
            # self.Thread_StopGrabacion.start()
            self._FLAG_Habilitar_Continuar = 0
        else:
            self.ui.visImu.append(
                "<font color='black'>>Los sensores todavia no estan emitiendo datos...</font>")

    def BUTTON_INFO_pressed(self):
        if (self._FLAG_Habilitar_Grabacion == 1):
            self.Thread_InfoGrabacion = InfoReader(self)
            self.Thread_InfoGrabacion.connection_status_signal.connect(
                self.ui.visImu.append)
            self.Thread_InfoGrabacion.start()
        else:
            self.ui.visImu.append(
                "<font color='black'>>Los sensores todavia no estan emitiendo datos...</font>")

    def start_sensor_threads(self):
        self._THREAD_imuDataReader.start()
        # self._THREAD_imuLidarReader.start()
        # self._THREAD_JSONMID360DataReader.start()
        self._THREAD_CPPJSONMID360.start()
        self._THREAD_MID360Reader.start()
        self._THREAD_EncoderLidarReader.start()

        self.ui.visImu.append(
            "<font color='green'>> Sensores funcionando asegurese que se puedan visualizar... </font>")

    def BUTTON_ON_pressed(self):
        if (self._FLAG_Habilitar_Robot == 1):
            if self._FLAG_buttonOnPressed:
                logging.info("Boton ON ya presionado...")
                self.ui.visImu.append(
                    "<font color='black'>> Boton ON ya presionado... </font>")
                return
            """ Iniciamos el robot """
            # Habilitamos flag
            self._Thread_Robot = RobotReader(self)
            self._Thread_Robot.start()
            self._THREAD_socketReceptionData.start()
            self._FLAG_buttonOnPressed = True
            logging.info("Estableciendo conexion con el Robot")
            # Establecemos conexión con el socket y la transmisión de los comandos del joystick
            # self._THREAD_socketReceptionData.start()
            while not (self._FLAG_socketConected):
                logging.info("esperando conexión con el robot")
                time.sleep(1)
            logging.info("Conectado!")
            """ Configuramos camaras """
            # Define the codec and create VideoWriter object
            logging.info("Establecemos los VideoWriters")
            fourcc = cv2.VideoWriter_fourcc(
                *'mp4v')  # Be sure to use lower case
            frameRate = self._VAR_FRAME_RATE

            # nombreDirVid=os.path.join(MY_FILEPATH,"videos","video_TumiBot_cam1_{}.mp4".format(self._VAR_counterVideoFiles))
            # self._VAR_outVideoWrite_list[0] = cv2.VideoWriter(nombreDirVid, fourcc, frameRate, (self._VAR_WIDTH_FRAME_CAM, self._VAR_HEIGHT_FRAME_CAM))

            nombreDirVid = os.path.join(
                MY_FILEPATH, "videos", "video_TumiBot_cam2_{}.mp4".format(self._VAR_counterVideoFiles))
            self._VAR_outVideoWrite_list[1] = cv2.VideoWriter(
                nombreDirVid, fourcc, frameRate, (self._VAR_WIDTH_FRAME_CAM, self._VAR_HEIGHT_FRAME_CAM))
            self._THREAD_updateCamera1.start()
            self._THREAD_updateCamera2.start()
            self._THREAD_updateCamera3.start()
        else:
            self.ui.visImu.append(
                "<font color='black'>>Espere, a una conexión con el ROBOT...</font>")
            self._FLAG_SENSORES_pressed = False

    def BUTTON_OFF_pressed(self):
        self.close()
        return
        logging.info("Cerrando conexión Robot")
        # Detenemos hilos de camara
        self._THREAD_updateCamera1.stop()
        self.ui.label_camMain.clear()
        self._THREAD_updateCamera2.stop()
        self.ui.label_camAruco.clear()
        logging.info("Se detuvo Cámaras")
        # Detenemos hilo de Recepcion de datos
        self._THREAD_socketReceptionData.stop()
        logging.info("Se detuvo recepción de datos")
        # Habilitamos el comboBox
        self.ui.ComboBox_Seleccionador_Robot.setEnabled(True)
        """ Reiniciamos los hilos cerrados """
        # Crea el socket de comunicacion con el robot
        self._THREAD_socketReceptionData = socketReception(self)
        self._THREAD_socketReceptionData._SIGNAL_tramaDatos.connect(
            self.updateIndicators)
        self._THREAD_socketTranssmisionData = threading.Thread(
            target=self.socketTransmissionData,)
        # Camaras
        self._THREAD_updateCamera1 = Camera(self, 1)
        self._THREAD_updateCamera1.change_pixmap_signal.connect(
            self.update_image1)
        self._THREAD_updateCamera2 = Camera(self, 2)
        self._THREAD_updateCamera2.change_pixmap_signal.connect(
            self.update_image2)
        # socket
        self._VAR_socketClient = None
        # joystick command
        self._VAR_joystickCommand = ""
        # Video counter
        self._VAR_counterVideoFiles = 0
        # List that contains the VideoWriter to MP4 of the video
        try:
            self._VAR_outVideoWrite_list[1].release()
            self._VAR_outVideoWrite_list[0].release()
        except:
            pass
        self._VAR_outVideoWrite_list = [None, None]
        logging.info("Desconectado!")
        self._FLAG_buttonOnPressed = False

    def TOGGLE_MODE(self):
        global FLAG_AUTO
        if not FLAG_AUTO:
            self.ui.label_mode.setText("Modo Autonomo")
            pixmap = QPixmap("iconos/automode.svg")
            pixmap = pixmap.scaled(60, 60,
                                   QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                                   QtCore.Qt.TransformationMode.SmoothTransformation)
            icon = QIcon(pixmap)
            self.ui.btn_toggle.setIcon(icon)
            self.ui.btn_toggle.setIconSize(QtCore.QSize(60, 60))

            if self._FLAG_socketConected:
                # enviamos el comando a traves del socket
                self._VAR_socketClient.sendall(pickle.dumps("$OAX2A1"))

            FLAG_AUTO = True
        else:
            self.ui.label_mode.setText("Modo Teleoperado")
            pixmap = QPixmap("iconos/Toggle.svg")
            pixmap = pixmap.scaled(60, 60,
                                   QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                                   QtCore.Qt.TransformationMode.SmoothTransformation)
            icon = QIcon(pixmap)
            self.ui.btn_toggle.setIcon(icon)
            self.ui.btn_toggle.setIconSize(QtCore.QSize(60, 60))
            if self._FLAG_socketConected:
                # enviamos el comando a traves del socket
                self._VAR_socketClient.sendall(pickle.dumps("$OAX2A0"))
            FLAG_AUTO = False

    def toggle_image_size(self, button):
        full_screen_window = FullScreenImage(button.image_path, self)
        full_screen_window.exec_()

    def load_images_galery(self, folder_path="imagenes", isAlbum=False):
        if not isAlbum:
            self.ui.btn_fotos.setStyleSheet(
                "QPushButton { color: black; background-color: #EA6C36; border-radius:20px; }")
            self.ui.btn_album.setStyleSheet(
                "QPushButton { color: white; background-color: #0F0F0F; border-radius:20px; }")
        else:
            self.ui.btn_album.setStyleSheet(
                "QPushButton { color: black; background-color: #EA6C36; border-radius:20px; }")
            self.ui.btn_fotos.setStyleSheet(
                "QPushButton { color: white; background-color: #0F0F0F; border-radius:20px; }")

        if not os.path.exists(folder_path):
            logging.info(f"La carpeta {folder_path} no existe.")
            return

        scroll_area = QScrollArea(self.ui.frame_galery)
        scroll_area.setWidgetResizable(True)

        container_widget = QWidget()
        layout = QGridLayout(container_widget)
        container_widget.setLayout(layout)
        scroll_area.setWidget(container_widget)

        def find_image_files(folder):
            image_files = []
            for root, _, files in os.walk(folder):
                for file in files:
                    if file.lower().endswith((".png", ".jpg", ".jpeg", ".gif", ".bmp")):
                        image_files.append(os.path.join(root, file))
            return image_files

        image_files = find_image_files(folder_path)

        row, col = 0, 0
        for i, file_path in enumerate(image_files):
            logging.debug(f"Procesando archivo: {file_path}")

            pixmap = QPixmap(file_path)
            if pixmap.isNull():
                logging.error(f"No se pudo cargar la imagen: {file_path}")
                continue

            if col == 4:
                col = 0
                row += 1

            frame = QFrame()
            frame.setMinimumSize(260, 260)
            frame.setMaximumSize(260, 260)

            layout_frame = QVBoxLayout()
            button = QPushButton()
            button.setMinimumSize(250, 250)
            button.setMaximumSize(250, 250)

            pixmap = pixmap.scaled(
                250, 250, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            icon = QIcon(pixmap)
            button.setIcon(icon)
            button.setIconSize(QSize(250, 250))

            button.image_path = file_path
            button.clicked.connect(
                lambda checked, btn=button: self.toggle_image_size(btn))

            layout_frame.addWidget(button)
            frame.setLayout(layout_frame)
            layout.addWidget(frame, row, col, 1, 1)
            col += 1

        if not hasattr(self.ui, 'frame_galery') or not self.ui.frame_galery:
            self.ui.frame_galery = QFrame(self)
            main_layout = QVBoxLayout(self.ui.frame_galery)
            self.ui.frame_galery.setLayout(main_layout)
        else:
            main_layout = self.ui.frame_galery.layout()

        while main_layout.count():
            item = main_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        main_layout.addWidget(scroll_area)

    # Función para cambiar el tamaño de la imagen

    def depuracion(self):
        global _FLAG_CONECT

        message = f"{self.ui.textEdit.toPlainText()}"

        try:
            # Enviar datos
            print(f"Enviando: {message}")
            self._VAR_socketClient.sendall(pickle.dumps(message))
            # Buscar respuesta
            data = None
            data = self._VAR_socketClient.recv(4096)
            print(f"La data del depurador es: {data}")
            data = pickle.loads(data)

            if data:

                print(f"Recibido: {data}")
                data_n = f'{data}'
                self.ui.label_37.setText(data_n)
                _FLAG_CONECT = False

            else:
                _FLAG_CONECT = False
                self.ui.label_37.setText("Es none")
                print("Es None")
        except IndexError:
            print("No llegó un dato anguloso")

        except:
            print("Cerrando socket")
            self._VAR_socketClient.close()
            _FLAG_CONECT = False

    def initialize_map_frames(self):
        for index, row in self.map_df.iterrows():
            name = row['name']
            image_path = row['photo']
            area = row['area']
            self.create_map_frame(name, area, image_path)

    def create_map_frame(self, name, area, image_path):
        frame = QFrame()
        frame.setMinimumSize(260, 350)
        frame.setMaximumSize(260, 350)

        layout = QVBoxLayout()
        button = QPushButton()
        button.setMinimumSize(250, 250)
        button.setMaximumSize(250, 250)
        pixmap = QPixmap(image_path)
        pixmap = pixmap.scaled(
            250, 250, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        icon = QIcon(pixmap)
        button.setIcon(icon)
        button.setIconSize(QtCore.QSize(250, 250))

        label_name = QLabel(str(name))  # Convertir a str si es necesario
        label_area = QLabel(str(area))  # Convertir a str si es necesario

        font = QFont()
        font.setFamily('MS Shell Dlg 2')
        font.setPointSize(14)

        label_name.setFont(font)
        label_area.setFont(font)

        label_name.setStyleSheet("color: white; text-align: center;")
        label_area.setStyleSheet("color: white; text-align: center;")
        label_name.setAlignment(QtCore.Qt.AlignCenter)
        label_area.setAlignment(QtCore.Qt.AlignCenter)

        # Set maximum width for labels
        label_name.setWordWrap(True)
        label_area.setWordWrap(True)

        layout.addWidget(button)
        layout.addWidget(label_name)
        layout.addWidget(label_area)
        frame.setLayout(layout)
        button.clicked.connect(lambda checked, user=name, area_map=area,
                               img=image_path: self.GoGeneral(user, area_map, img))
        # Insert frame to the left of frame_mapgen
        layout_usuarios = self.ui.frame_mapas_list.layout()
        index_create = layout_usuarios.indexOf(self.ui.frame_mapgen)
        layout_usuarios.insertWidget(index_create, frame)

    def GoGenerarReport(self):
        # Frames Generales
        # Contiene frame_left_up y frame_left_down
        self.ui.main_body_left.setMaximumSize(10000, 100000)
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(0, 0)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(0, 0)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(10000, 10000)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_general.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(0, 0)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(10000, 10000)
        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)

    def GoEstadisticas(self):
        # Frames Generales
        # Contiene frame_left_up y frame_left_down
        self.ui.main_body_left.setMaximumSize(10000, 100000)
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(0, 0)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(0, 0)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(10000, 10000)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_general.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(0, 0)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(10000, 10000)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)
    # Funcion que distribuye funcion de posicion, control y angulos

    def distribuir_secret(self):

        self.movie = QMovie(os.path.join("iconos", "marcianito.gif"))
        self.ui.label_gif.setMovie(self.movie)
        self.ui.label_gif.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.movie.setScaledSize(QSize().scaled(2300, 780,
                                                Qt.AspectRatioMode.KeepAspectRatio))
        # self.label_gif.setMaximumSize(100000000, 10000000)
        self.movie.start()

        # Frames Generales
        # Contiene frame_left_up
        self.ui.main_body_left.setMaximumSize(10000, 10000)
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(0, 0)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(2000, 1240)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(0, 0)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(100000, 100000)
        self.ui.frame_general.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(0, 0)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)
    #################################################################

    def distribuir_arm(self):

        # Frames Generales
        self.ui.main_body_left.setMaximumSize(
            10000, 1100)  # Contiene frame_left_up
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(0, 0)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(2000, 1240)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(0, 0)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_general.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(10000, 10000)
        self.ui.frame_posicion.setMaximumSize(0, 0)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)
    #################################################################

    def distribuir_general(self):
        global FLAG_MAPA
        # Frames Generales
        self.ui.main_body_left.setMaximumSize(0, 0)  # Contiene frame_left_up
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(10000, 10000)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(0, 0)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(0, 0)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(10000, 10040)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)
        if not FLAG_MAPA:
            self.ui.frame_mapa.setMaximumSize(100000, 100000)
            self.ui.frame_general.setMaximumSize(0, 0)
            self.ui.frame_general_auto.setMaximumSize(0, 0)
        else:

            # self.btn_mapa2.setIcon(icon)
            # self.btn_mapa2.setIconSize(QtCore.QSize(400, 250))
            if not FLAG_AUTO:
                self.ui.frame_mapa.setMaximumSize(0, 0)
                self.ui.frame_general.setMaximumSize(10000, 10000)
                self.ui.frame_general_auto.setMaximumSize(0, 0)
            else:
                self.ui.frame_mapa.setMaximumSize(0, 0)
                self.ui.frame_general.setMaximumSize(0, 0)
                self.ui.frame_general_auto.setMaximumSize(10000, 10000)
                # self.btn_VisionCamara1_3.lower()
                # self.setLayout(self.frame_cam_map)
                # self.frame_cam_map().removeWidget(self.btn_VisionCamara1_3)
                # self.frame_cam_map().removeWidget(self.btn_map)
                # self.frame_cam_map().addWidget(self.btn_map)
                # self.frame_cam_map().addWidget(self.btn_VisionCamara1_3)

    #################################################################

    def distribuir_document(self):

        # Frames Generales
        # Contiene frame_left_up
        self.ui.main_body_left.setMaximumSize(10000, 1100)
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(0, 0)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(0, 0)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(10000, 10000)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_general.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(0, 0)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(100000, 100000)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)
    #################################################################
    # Funcion que distribuye funcion de camara

    def distribuir_posicion(self):

        # Frames Generales
        # Contiene frame_left_up
        self.ui.main_body_left.setMaximumSize(10000, 1100)
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(0, 0)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(2000, 1240)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(0, 0)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_general.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(10000, 10040)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)
        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)
###############################################

    def GoGeneral(self, name, area_map, img):
        global FLAG_MAPA, MAP_NAME, MAP_AREA, MAP_PIC
        FLAG_MAPA = True
        MAP_NAME = name
        MAP_AREA = area_map
        MAP_PIC = img

        pixmap = QPixmap(MAP_PIC)
        pixmap = pixmap.scaled(400, 250,
                               QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                               QtCore.Qt.TransformationMode.SmoothTransformation)
        icon = QIcon(pixmap)
        self.ui.btn_mapa.setIcon(icon)
        self.ui.btn_mapa.setIconSize(QtCore.QSize(400, 250))
        pixmap = QPixmap(MAP_PIC)
        pixmap = pixmap.scaled(400, 250,
                               QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                               QtCore.Qt.TransformationMode.SmoothTransformation)
        icon = QIcon(pixmap)
        self.ui.btn_mapa_2.setIcon(icon)
        self.ui.btn_mapa_2.setIconSize(QtCore.QSize(400, 250))

        # Frames Generales
        self.ui.main_body_left.setMaximumSize(0, 0)  # Contiene frame_left_up
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(10000, 10000)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(0, 0)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(0, 0)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_general.setMaximumSize(10000, 10000)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(10000, 10040)
        self.ui.frame_arch.setMaximumSize(0, 0)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)

        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)
    # Funcion que distribuye funcion de archivos

    def distribuir_archivos(self):

        # Frames Generales
        # Contiene frame_left_up
        self.ui.main_body_left.setMaximumSize(10000, 1100)
        # Contiene a frame_general y a frame_mapa
        self.ui.main_frames_login.setMaximumSize(0, 0)
        # Contiene a frame_arm, frame_position y frame_secret
        self.ui.frame_left_up.setMaximumSize(0, 0)
        # Contiene a frame_arch, frame_documents y derivados del reporte
        self.ui.frame_left_down.setMaximumSize(100000, 10000)

        # Frames Ventanas
        self.ui.frame_first.setMaximumSize(0, 0)
        self.ui.frame_mapa.setMaximumSize(0, 0)
        self.ui.frame_secret.setMaximumSize(0, 0)
        self.ui.frame_general.setMaximumSize(0, 0)
        self.ui.frame_arm.setMaximumSize(0, 0)
        self.ui.frame_posicion.setMaximumSize(0, 0)
        self.ui.frame_arch.setMaximumSize(10000, 10000)

        # Frames Reportes
        self.ui.frame_menu_report.setMaximumSize(0, 0)
        self.ui.frame_estadisticas.setMaximumSize(0, 0)
        self.ui.frame_generar_report.setMaximumSize(0, 0)

        # Labels Camaras
        # self.ui.VisionCamara1.setMinimumSize(960,570)
        # self.ui.VisionCamara1.setMaximumSize(960,570)
        # self.ui.VisionCamara2.setMinimumSize(250, 150)
        # self.ui.VisionCamara2.setMaximumSize(250, 150)

        self.load_images_galery()
    # Funcion para reiniciar distribucion de espacios

    @pyqtSlot(np.ndarray)
    def update_image1(self, cv_img):
        """Updates the image_label with a new opencv image"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height,
                         width * channel, QImage.Format.Format_RGB888)
        scaledImage = q_image.scaled(self.ui.label_camMain.size(),
                                     aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio)
        pixMap = QPixmap.fromImage(scaledImage)
        self.ui.label_camMain.setPixmap(pixMap)
        self.ui.label_camMain.setAlignment(Qt.AlignmentFlag.AlignCenter)

        scaledImage1 = q_image.scaled(self.ui.PTZ_Vision.size(),
                                      aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio)
        pixMap1 = QPixmap.fromImage(scaledImage1)
        self.ui.PTZ_Vision.setPixmap(pixMap1)
        self.ui.PTZ_Vision.setAlignment(Qt.AlignmentFlag.AlignCenter)

    @pyqtSlot(np.ndarray)
    def update_image2(self, cv_img):
        """Updates the image_label with a new opencv image"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height,
                         width * channel, QImage.Format.Format_RGB888)
        scaledImage = q_image.scaled(self.ui.label_camAruco.size(),
                                     aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio)
        pixMap = QPixmap.fromImage(scaledImage)
        self.ui.label_camAruco.setPixmap(pixMap)
        self.ui.label_camAruco.setAlignment(Qt.AlignmentFlag.AlignCenter)

    @pyqtSlot(np.ndarray)
    def update_image3(self, cv_img):
        """Updates the image_label with a new opencv image"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height,
                         width * channel, QImage.Format.Format_RGB888)
        scaledImage = q_image.scaled(self.ui.label_camPost.size(),
                                     aspectRatioMode=Qt.AspectRatioMode.KeepAspectRatio)
        pixMap = QPixmap.fromImage(scaledImage)
        # print("update image 3")
        self.ui.label_camPost.setPixmap(pixMap)
        self.ui.label_camPost.setAlignment(Qt.AlignmentFlag.AlignCenter)

    def closeEvent(self, event):
        event.accept()
        logging.info("closing event")
        try:
            self._VAR_outVideoWrite_list[1].release()
            self._VAR_outVideoWrite_list[0].release()
        except:
            pass
        self._VAR_outVideoWrite_list = [None, None]
        time.sleep(1)
        os._exit(0)


def waitForLoading():
    global _FLAG_continue  # ,mesh_item
    t_start = time.time()
    """Creamos el directorio para videos """
    temp_videosFilePath = os.path.join(MY_FILEPATH, "videos")
    if not os.path.exists(temp_videosFilePath):
        # Creamos el directorio
        os.mkdir(temp_videosFilePath)
        logging.info(f"Se ha creado el directorio: {temp_videosFilePath}")
    else:
        logging.info(f"Ya existe el directorio: {temp_videosFilePath}")

    # try:
    #    """ Cargamos el archivo MESH robot """
        # Cargar el archivo STL

    #    stl_filename = os.path.join(MY_FILEPATH,'robotMesh.stl')
    #    stl_mesh = mesh.Mesh.from_file(stl_filename)

        # Obtener los vértices y las caras del archivo STL
    #    vertices = stl_mesh.vectors.reshape(-1, 3)
    #    faces = np.arange(len(vertices), dtype=int).reshape(-1, stl_mesh.vectors.shape[-1])

        # Crear un objeto GLMeshItem y agregarlo a la vista
    #    mesh_item = gl.GLMeshItem(vertexes=vertices, edge_color=[0, 0, 0, 1.0],faces=faces, smooth=False, drawEdges=True)
    #    mesh_item.setGLOptions('opaque')  # Opción para bordes intensos
    # except:
    #    logging.info("archivo Robot-Mesh no encontrado")
    #    vertices = np.array([
    #        [0, 0, 0],   # Vértice 0
    #        [1, 0, 0],   # Vértice 1
    #        [0, 1, 0],   # Vértice 2
    #        [0, 0, 1]    # Vértice 3
    #    ])

    #    faces = np.array([
    #        [0, 1, 2],   # Cara 0 (triángulo)
    #        [0, 2, 3]    # Cara 1 (triángulo)
    #    ])

    #    meshdata = gl.MeshData(vertexes=vertices, faces=faces)
    #    mesh_item = gl.GLMeshItem(meshdata=meshdata, color=(1, 0, 0, 1), smooth=False)

    """ Verificamos que todo haya ocurrido correctamente """
    while time.time()-t_start < 2.5:
        pass
    _FLAG_continue = True


def setUpGUI():
    global _FLAG_continue
    # Creamos la app
    app = QApplication(sys.argv)
    # Cargar la imagen para el splash screen
    # import pics_rc
    splash_image = QPixmap("iconos/loadScreen.png")
    splash_image = splash_image.scaled(800, 600,
                                       QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                                       QtCore.Qt.TransformationMode.SmoothTransformation)
    splash = QSplashScreen(splash_image)
    splash.show()
    _FLAG_continue = False
    th = threading.Thread(target=waitForLoading,)
    th.start()
    while not (_FLAG_continue):
        pass
    splash.close()
    # Creamos el visualizador
    GUI_window = GUI()
    logging.info("______________INIT GUI____________")
    GUI_window.show()
    sys.exit(app.exec_())
