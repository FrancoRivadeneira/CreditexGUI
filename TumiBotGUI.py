
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
from utils.jsonReception import jsonReception
from utils.lidarDataReader import lidarDataReader
from utils.Mid360DataReader import Mid360DataReader
from utils.RobotNetworkCheckThread import RobotNetworkCheckThread
from utils.RobotReader import RobotReader
from utils.socketReception import socketReception
from utils.StopReader import StopReader
from utils.HoraUpdater import HoraUpdater
from utils.DateUpdater import DateUpdater
from MainWindow import Ui_MainWindow

## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import cv2 ## For Camera visualization  ###
import sys 
import os 
import logging ## Libreria para info Logging
import numpy as np             ###
import time
import threading ## For hilos
from pygame.locals import *    ###
import pickle
#import paramiko
# import pyqtgraph.opengl as gl  ###
import pyqtgraph as pg         ###
from PyQt5.uic import loadUi


""" COMANDOS PARA LA NUCLEO"""

comandoList=[ ["$OAX3J0A",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0],"Avanzar"],
              ["$OAX3J0B",[0,1,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0],"Retroceder"],
              ["$OAX3J0r",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1],"Giro Horario"],
              ["$OAX3J0l",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 1,0],"Giro Antihorario"],
              ["$OAX3JR1",[0,0,0,0,1, 0,0,0,0,0, 0,1,0,0,0, 0,0],"Prender Luces"],
              ["$OAX3JR2",[0,0,0,0,1, 0,0,0,0,0, 0,0,1,0,0, 0,0],"Apagar Luces"],
              ["$OAX3JX1",[0,0,1,0,0, 0,0,0,0,0, 0,0,0,1,0, 0,0],"Aumentar Velocidad"],
              ["$OAX3JX2",[0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,1, 0,0],"Disminuir Velocidad"],              
              ["$OAX3JRA",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1],"Avanzar Derecha"],
              ["$OAX3JLA",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 1,0],"Avanzar Izquierda"],
              ["$OAX1M"  ,[0,0,0,1,0, 0,0,0,0,0, 0,0,0,1,0, 0,0],"Activar Motores"], ### CAMBIAR ESTO es Y + FlechaArriba
              ["$OAX3JAR",[0,0,0,1,0, 0,0,0,0,0, 0,0,0,0,1, 0,0],"Apagar Rele"], ## CAMBIAR ESTO es Y + FlechaAbajo
              ["$OAX3JBE",[0,0,0,1,0, 0,0,0,0,0, 0,0,1,0,0, 0,0],"Boton Emergencia"]] ## CAMBIAR ESTO es Y + FlechaIzquierda

## Aplicamos transpuesta a la lista de comandos para hacer uso del built function "index"

""" LOGGIN FORMAT """
# Creamos el formato del log
FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
logging.basicConfig(level=logging.INFO, format=FORMAT)

## My system enviroment
MY_FILEPATH= os.path.dirname(os.path.abspath(__file__))

#joystick
EVENT_TYPE_LEFT_TRIGGER=4
if sys.platform=="linux":
    EVENT_TYPE_LEFT_TRIGGER=2

## ROBOT INFO (RASPBERRY REMOTE)
# ROBOT_TERRESTRE_IP_ADDR=  "192.168.1.166" #"192.168.1.166""192.168.1.133"
ROBOT_TERRESTRE_IP_ADDR=  "10.100.110.26" #"192.168.1.166""192.168.1.133"
ROBOT_TERRESTRE_USERNAME= "pi"
ROBOT_TERRESTRE_PASSWORD = "raspberry"
puerto = 22  # Puerto por defecto de SSH
ROBOT_TERRESTRE_PORT=8666
ROBOT_TERRESTRE_CAMS_PORT_LIST=[8080,8088]
ROBOT_TERRESTRE_FRAME_RATE=10
ROBOT_TERRESTRE_WIDTH_FRAME_CAM=640
ROBOT_TERRESTRE_HEIGHT_FRAME_CAM=480
ROBOT_TERRESTRE_MIN_VALOR_BATERIA=22
ROBOT_TERRESTRE_RANGO_BATERIA=25-22

## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"

class GUI(QMainWindow):
    def __init__(self):
        #global mesh_item 
        QMainWindow.__init__(self)
        """ Configuramos GUI"""
        ## Creamos nuestra GUI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowFlags(Qt.FramelessWindowHint)
                 
        """ DEFINICION de FLAG """
        self._FLAG_buttonOnPressed = False
        self._FLAG_socketConected = False
        self._FLAG_sshImuConnected=False
        self._FLAG_sshLidarConnected=False
        self._FLAG_sshMID360Connected=False

        self._sshRobotConnected = False
        self._sshCajaSensoresConnected = False
        self._sshGrabacionConnected = False
        self._sshStopConnected = False
        
        self._FLAG_LidarFuncionando=False
        
        self._FLAG_MID360Funcionando=False

        self._FLAG_sshEncoderConnected=False
        self._FLAG_EncoderFuncionando=False

        self._FLAG_SOCKET_ROBOT_CONNECTED=False

        self._FLAG_Habilitar_Robot    = 0
        self._FLAG_Habilitar_Sensores = 0
        self._FLAG_SENSORES_pressed = False
        self._FLAG_Habilitar_Grabacion = 0
        self._Flag_BotonGrabar_pressed = False
        self._FLAG_Habilitar_Continuar = 0

        """ DEFINICION de Variables """
        # Variables dependientes del tipo del Robot
        self._VAR_USERNAME=ROBOT_TERRESTRE_USERNAME
        self._VAR_PASSWORD=ROBOT_TERRESTRE_PASSWORD
        self._VAR_IP_ADDR=ROBOT_TERRESTRE_IP_ADDR
        self._VAR_PORT=ROBOT_TERRESTRE_PORT
        self._VAR_CAMS_PORT_LIST=ROBOT_TERRESTRE_CAMS_PORT_LIST
        self._VAR_FRAME_RATE=ROBOT_TERRESTRE_FRAME_RATE
        self._VAR_WIDTH_FRAME_CAM=ROBOT_TERRESTRE_WIDTH_FRAME_CAM
        self._VAR_HEIGHT_FRAME_CAM=ROBOT_TERRESTRE_HEIGHT_FRAME_CAM
        self._VAR_MIN_VALOR_BATERIA=ROBOT_TERRESTRE_MIN_VALOR_BATERIA
        self._VAR_RANGO_BATERIA=ROBOT_TERRESTRE_RANGO_BATERIA


        self._VAR_Nombre_GRabacion = ""
        # socket
        self._VAR_socketClient= None

        self._VAR_socketClienteROBOT=None
        # Video counter
        self._VAR_counterVideoFiles=0
        # List that contains the VideoWriter to MP4 of the video
        self._VAR_outVideoWrite_list=[None,None]
        self._VAR_tramaDatos=""
        # Objeto Mesh
        #self._VAR_meshRobot=mesh_item
        # Objetos detector internet:
        self._Var_valueCajaSensores = 0
        self._Var_valueRobot = 0

        """  Realizamos el conteo de videos en la computadora """
        temp_videosFilePath=os.path.join(MY_FILEPATH,"videos")
        self._VAR_counterVideoFiles=len(os.listdir(temp_videosFilePath))

        """ Cargamos los visualizadores"""
        #Esto es de cuando el lidar se visualizaba en 2D
        #grid_item = gl.GLGridItem()
        #self.ui.visLidar.addItem(grid_item)

        # #Esto es de cuando el lidar se visualizaba en 3D
        # self.scatter = gl.GLScatterPlotItem(
        #     pos=np.zeros((1, 3)),  # Inicialmente vacío
        #     size=2,
        #     color=(1, 1, 1, 1),  # Color blanco
        #     pxMode=True)
        # self.ui.visLidar.addItem(self.scatter)

        #self.ui.visImu.addItem(self._VAR_meshRobot)             
        #self.ui.visImu.addItem(grid_item)
        
        ## >>> Visualizador Roll
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
        self._THREAD_RobotNetwork_thread.connection_status_signal.connect(self.ui.visImu.append)
        self._THREAD_RobotNetwork_thread.connection_status.connect(self.Robot_update_connection_status)
        # verificacion de la red de internet de la caja de sensores
        self._THREAD_CajaSensoresNetwork_thread = CajaSensoresNetworkCheckThread(self)
        self._THREAD_CajaSensoresNetwork_thread.connection_status_signal.connect(self.ui.visImu.append)
        self._THREAD_CajaSensoresNetwork_thread.connection_status.connect(self.Sensor_update_connection_status)
        

        # Crea el socket de comunicacion y recepcion de datos con el robot
        self._THREAD_socketReceptionData=socketReception(self)
        #self._THREAD_socketReceptionData._SIGNAL_msg.connect(self.ui.visImu.append)
        self._THREAD_socketReceptionData._SIGNAL_tramaDatos.connect(self.updateIndicators)
        # Check the joystick connection status and update command
        self._THREAD_joytickCommand=JoystickControl(self)
        self._THREAD_joytickCommand._SIGNAL_command.connect(self.sendJoystickCommand)
        # self._THREAD_joytickCommand._SIGNAL_updateJoystickIndicator.connect(self.ui.progressBar_mando.setValue)
        # Camaras
        self._THREAD_updateCamera1=Camera(self,1)
        self._THREAD_updateCamera1.change_pixmap_signal.connect(self.update_image1)

        self._THREAD_updateCamera2=Camera(self,2)
        self._THREAD_updateCamera2.change_pixmap_signal.connect(self.update_image2)
        # SSH Imu
        self._THREAD_imuDataReader=imuDataReader(self)
        self._THREAD_imuDataReader._SIGNAL_data.connect(self.updateMesh)

        #JSON Lidar 
        #self._THREAD_JSONMID360DataReader = jsonReception(self)
        #self._THREAD_JSONMID360DataReader._SIGNAL_data.connect(self.updateLidar3D)

        #Json Lidar cpp

        self._THREAD_CPPJSONMID360 = cppjsonReception(self)
        self._THREAD_CPPJSONMID360._SIGNAL_data.connect(self.updateLidar3D)
        #self._THREAD_CPPJSONMID360.start()

        # SSH Lidar  se solia usar para el ydlidar su visualizador solia ser visLidar
        #self._THREAD_imuLidarReader=lidarDataReader(self)
        #self._THREAD_imuLidarReader._SIGNAL_data.connect(self.updateLidar)

        # SSH MID360
        self._THREAD_MID360Reader=Mid360DataReader(self)
        self._THREAD_MID360Reader._SIGNAL_data.connect(self.updateMID360)
        
        # SSH Encoder
        self._THREAD_EncoderLidarReader=EncoderDataReader(self)
        self._THREAD_EncoderLidarReader._SIGNAL_data.connect(self.updateEncoder)

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
        self._TIMER_updateMesh=QTimer()
        self._TIMER_updateMesh.timeout.connect(self.updateMesh)

        # Robot Lidar
        #self._TIMER_updateLidar=QTimer()
        #self._TIMER_updateLidar.timeout.connect(self.updateLidar)

        """  Conectamos los eventos de los objetos con las funciones """
        self.ui.Encender_button.pressed.connect(self.BUTTON_ON_pressed)
        # self.ui.Apagar_button.pressed.connect(self.BUTTON_OFF_pressed)
        # self.ui.Sensores_button.pressed.connect(self.BUTTON_SENSORS_pressed)
        self.ui.button_Grabar.pressed.connect(self.BUTTON_GRABAR_pressed)
        self.ui.button_stop.pressed.connect(self.BUTTON_STOP_pressed)
        self.ui.button_info.pressed.connect(self.BUTTON_INFO_pressed)

        """ Iniciamos hilos o timers """
        self._THREAD_RobotNetwork_thread.start()
        self._THREAD_CajaSensoresNetwork_thread.start()
        #self._THREAD_socketReceptionData.start()

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
            self.ui.button_Grabar.setStyleSheet("""QPushButton {
                    font-weight: bold;  /* Siempre en negrita */
                }
            """)
            
    def updateMesh(self,data):
        # Definimo la matriz de transformación
        transform = QMatrix4x4()
        #print(data)
        """ Roll """
        angle=data[0]+180
        ## >> vis IMU
        # Realizamos la rotacion al Robot Mesh
        transform.rotate(angle, 1, 0, 0)
        ## >> vis Roll
        # Matrix de rotacion
        rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                                    [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])
        # Rotar los puntos del cuadrado
        rollSquarePoints = np.dot(self._VAR_rollSquarePoints, rotation_matrix)
        # Rotar los puntos de la línea
        rollLinePoints = np.dot(self._VAR_rollLinePoints, rotation_matrix)
        # Actualizar los datos en los trazados
        self._VAR_rollReference_Square.setData(rollSquarePoints[:, 0], rollSquarePoints[:, 1])
        self._VAR_rollReference_Line.setData(rollLinePoints[:, 0], rollLinePoints[:, 1])
        self.ui.label_visRoll.setText(f'Roll: <span style="color: #90EE90;">{angle-180:.1f}°</span>')
        """ Pitch """
        angle=data[1]+180
        ## >> vis IMU
        # Realizamos la rotacion al Robot Mesh
        transform.rotate(angle, 0, 1, 0)
        ## >> vis Pitch
        # Matrix de rotacion
        rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                                    [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])
        # Rotar los puntos del cuadrado
        pitchSquarePoints = np.dot(self._VAR_pitchSquarePoints, rotation_matrix)
        # Rotar los puntos de la línea
        pitchLinePoints = np.dot(self._VAR_pitchLinePoints, rotation_matrix)
        # Actualizar los datos en los trazados
        self._VAR_pitchReference_Square.setData(pitchSquarePoints[:, 0], pitchSquarePoints[:, 1])
        self._VAR_pitchReference_Line.setData(pitchLinePoints[:, 0], pitchLinePoints[:, 1])
        self.ui.label_visPitch.setText(f'Pitch: <span style="color: #90EE90;">{angle-180:.1f}°</span>')
        """ Yaw """
        angle=data[2]+180
        #>> vis IMU
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
        self._VAR_yawReference_Square.setData(yawSquarePoints[:, 0], yawSquarePoints[:, 1])
        self._VAR_yawReference_Line.setData(yawLinePoints[:, 0], yawLinePoints[:, 1])
        self.ui.label_visImuYaw.setText(f'Yaw: <span style="color: #90EE90;">{angle-180:.1f}°</span>')
        
        
        """ Actualizamos la transformacion del Mesh"""
        # Aplicar la nueva matriz de transformación al objeto
        #self._VAR_meshRobot.setTransform(transform)

    def updateLidar(self,data):
        #print("a")
        tempRanges=eval(data)
        #print(tempRanges)
        temp_numPoints=len(tempRanges)
        angle_min=-3.1415927410125732
        angle_max=3.1415927410125732
        angles=np.linspace(angle_min,angle_max,temp_numPoints)          #Se puede sumar un valor de angulo si el lidar estaba mal colocado
        # Obtenemos los valores de XY del anillo del Lidar
        tempX=tempRanges*np.cos(angles)
        tempY=tempRanges*np.sin(angles)
        pointCloud=(np.vstack((tempX,-tempY))).T
        ## Ploteamos en el Vis
        item = pg.ScatterPlotItem(pxMode=True, pos=pointCloud,size=2)
        self.ui.visLidar.clear()
        self.ui.visLidar.addItem(item)
        if self._FLAG_LidarFuncionando ==True:
            # self.ui.progressBar_lidar.setValue(100)
            pass
        else:
            # self.ui.progressBar_lidar.setValue(0)
            pass
        if data == None:
            # self.ui.progressBar_lidar.setValue(0)
            pass

    def updateLidar3D(self,data):
        self.scatter.setData(pos=data, size=0.1, color=(1, 1, 1, 1))
    
    def updateMID360(self,data):
        
        if self._FLAG_MID360Funcionando ==True:
            # self.ui.progressBar_lidar.setValue(100)
            pass
        else:
            # self.ui.progressBar_lidar.setValue(0)
            pass
        
        if data == None:
            # self.ui.progressBar_lidar.setValue(0)
            pass

    def updateHour(self,data):
        data=str(data)
        self.ui.horaLabel.setText(data)

    def updateDate(self,data):
        data=str(data)
        self.ui.dateLabel.setText(data)

    def updateEncoder(self, data):
        Velocidades=eval(data)
        texto_para_label = str(Velocidades)
        self.ui.encoder_data.setText(texto_para_label)                       # Self para mostrar la data de la velocidad
        # Verificar el contenido del mensaje y aplicar el color correspondiente
        if texto_para_label == "[0, 0, 0, 0, 0, 0, 0, 0]":
            self.ui.encoder_data.setStyleSheet("background-color: white;color: red")
        else:
            self.ui.encoder_data.setStyleSheet("background-color: white;color: green")
        
        
        if self._FLAG_EncoderFuncionando ==True:
            self.ui.progressBar_encoder.setValue(100)
        else:
            self.ui.progressBar_encoder.setValue(0)
            
        if data == None:
            self.ui.progressBar_encoder.setValue(0)
    
    def updateIndicators(self,tramaDatos):
        if tramaDatos=="" or not(self._FLAG_socketConected):
            return
        try:
            logging.info("Trama recibida: {}".format(tramaDatos[:-1]))
            comprobar = tramaDatos[0:6]
            if comprobar == '$OAX1j':   ###  $OAX1jb145s15l0m11r0d1p1
                bat = tramaDatos.find('b')
                speed = tramaDatos.find('s')
                lights = tramaDatos.find('l')
                motors = tramaDatos.find('m')
                lidar= tramaDatos.find('n')
                dust= tramaDatos.find('d')
                path= tramaDatos.find('p')
                #lidar= tramaDatos.find('r')
                
                ## b:140 170
                
                if (bat != -1) and (speed != -1):
                    valorBateria = tramaDatos[(bat+1):speed]
                    if valorBateria == 'e':
                        logging.info('Error')
                    else:
                        if valorBateria.isnumeric():
                            valorBateria = int(valorBateria)/10
                            self.ui.label_bateria.setText("Bateria: {:.1f} V".format(valorBateria))
                            porcentaje = (valorBateria - self._VAR_MIN_VALOR_BATERIA)/self._VAR_RANGO_BATERIA*100 # normalizamos el valor de la batería
                            if porcentaje < 0 :
                                self.ui.progressBar_bateria.setValue(0)
                            elif porcentaje > 100:
                                self.ui.progressBar_bateria.setValue(100)
                            else:
                                self.ui.progressBar_bateria.setValue(int(porcentaje))
                if (speed != -1) and (lights != -1):
                    valorVelocidad = tramaDatos[(speed+1):lights]
                    if valorVelocidad.isnumeric():
                        valorVelocidad = int(valorVelocidad)
                        self.ui.label_velocidad.setText("Velocidad:")
                        self.ui.progressBar_velocidad.setValue(valorVelocidad)
                if (lights != -1) and (motors != -1):
                    valorLights = tramaDatos[(lights+1):motors]
                    if valorLights.isnumeric():
                        valorLights = int(valorLights)
                        if valorLights == 1:
                            # self.ui.progressBar_luces.setValue(100)
                            pass
                        
                        if valorLights == 0:
                            # self.ui.progressBar_luces.setValue(0)
                            pass
                if (motors != -1):
                    valorMotores = tramaDatos[(motors+1):lidar] #Hay 4 motores
                    for id,motor in enumerate(valorMotores):
                        if motor.isnumeric():
                            motor = int(motor)
                            if motor == 0:
                                command = "self.ui.progressBar_motor{}.setValue(0)".format(id+1)
                            if motor == 1:
                                command = "self.ui.progressBar_motor{}.setValue(100)".format(id+1)
                            eval(command)
                if (lidar!=-1):
                    numCloud= tramaDatos[lidar+1:dust]
                    if numCloud.isnumeric():
                        #self.ui.textBrowser_lidar.setText(numCloud)
                        print("lidar cloud")
                if (dust != -1):
                    valorDust = tramaDatos[(dust+1):path]
                    if valorDust.isnumeric():
                        valorDust = int(valorDust)
                        if valorDust == 0:
                            command = "self.ui.progressBar_polvo.setValue(0)"                        
                        if valorDust == 1:
                            command = "self.ui.progressBar_polvo.setValue(100)"
                        eval(command)
                if (path != -1):
                    valorPath = tramaDatos[(path+1):-1]
                    if valorPath.isnumeric():
                        valorPath = int(valorPath)
                        if valorPath == 0:
                            command = "self.ui.progressBar_path.setValue(0)"                       
                        if valorPath == 1:
                            command = "self.ui.progressBar_path.setValue(100)"
                        eval(command)
                
                            
                    
        except Exception as e:
            logging.info('Datos erroneos! [ERROR:{}]'.format(e))

    def sendJoystickCommand(self,comando):
        try:
            if self._FLAG_socketConected:
                # enviamos el comando a traves del socket
                self._VAR_socketClient.sendall(pickle.dumps(comando))
                if comando== "$OAX3JAR": ## Apagar relé
                    ## Esperamos un tiempo adicional para volver a enviar comandos
                    time.sleep(2)
        except:
            logging.info("Error transmitiendo datos por el socket.")
            time.sleep(1)

    def BUTTON_SENSORS_pressed(self):
        if (self._FLAG_Habilitar_Sensores==1):
            
            if self._FLAG_SENSORES_pressed:
                self.ui.visImu.append("<font color='black'>> Boton sensores ya presionado </font>")
                return
            self._FLAG_Habilitar_Grabacion = 1  
            self.Thread_CajaSensores = CajaSensorReader(self)
            self.Thread_CajaSensores.start()
            self.ui.visImu.append("<font color='black'>> Los sensores han sido inicializados. Por favor, espere 40 segundos para que el proceso finalice.</font>")

            # Configura un QTimer para iniciar los demás hilos después de 60 segundos
            self.timer = QTimer(self)
            self.timer.setSingleShot(True)  # Asegura que el temporizador se ejecute solo una vez
            self.timer.timeout.connect(self.start_sensor_threads)  # Conecta al método que inicia los hilos
            self.timer.start(40000)  # 60000 milisegundos = 60 segundos

             
            self._FLAG_SENSORES_pressed = True  
        else:
            self.ui.visImu.append("<font color='black'>>Espere a una conexión con la Caja Sensores...</font>")
            self._FLAG_Habilitar_Grabacion = 0
            #self._FLAG_SENSORES_pressed = False

    def BUTTON_GRABAR_pressed(self):
        
        if (self._FLAG_Habilitar_Grabacion==1):
            
            
            dialog = CustomDialog()
            if dialog.exec_() == QDialog.Accepted:
                texto = dialog.line_edit.text().replace(' ', '_').replace('.', '_').replace(',', '_')
                self._VAR_Nombre_GRabacion = texto

                if self._FLAG_Habilitar_Continuar == 1:
                    self.ui.visImu.append("<font color='black'>>Ya esta presionado el boton grabacion</font>")
                    return
                

                self._FLAG_Habilitar_Continuar = 1

                self.ui.button_Grabar.setStyleSheet("""
                    QPushButton {
                        background-color: #e74c3c;  /* Rojo */
                        color: white;
                        font-weight: bold;
                    }""")

                self.Thread_CajaGrabacion = GrabarReader(self)
                self.Thread_CajaGrabacion.connection_status_signal.connect(self.ui.visImu.append)
                self.Thread_CajaGrabacion.start()
        
                
                
            else:
                self.ui.visImu.append("<font color='black'>>Se cancelo grabacion</font>")

        else:
            self.ui.visImu.append("<font color='black'>>Los sensores todavia no estan emitiendo datos...</font>")
    
    def BUTTON_STOP_pressed(self):
        if (self._FLAG_Habilitar_Grabacion==1):
            self.ui.button_Grabar.setStyleSheet("""QPushButton {
                    font-weight: bold;  /* Siempre en negrita */
                }""")
            self.Thread_StopGrabacion = StopReader(self)
            self.Thread_StopGrabacion.connection_status_signal.connect(self.ui.visImu.append)
            self.Thread_StopGrabacion.start()
            self._FLAG_Habilitar_Continuar = 0
        else:
            self.ui.visImu.append("<font color='black'>>Los sensores todavia no estan emitiendo datos...</font>")

    def BUTTON_INFO_pressed(self):
        if (self._FLAG_Habilitar_Grabacion==1):
            self.Thread_InfoGrabacion = InfoReader(self)
            self.Thread_InfoGrabacion.connection_status_signal.connect(self.ui.visImu.append)
            self.Thread_InfoGrabacion.start()
        else:
            self.ui.visImu.append("<font color='black'>>Los sensores todavia no estan emitiendo datos...</font>")

    def start_sensor_threads(self):
        self._THREAD_imuDataReader.start()
        #self._THREAD_imuLidarReader.start()
        #self._THREAD_JSONMID360DataReader.start()
        self._THREAD_CPPJSONMID360.start()
        self._THREAD_MID360Reader.start()
        self._THREAD_EncoderLidarReader.start()
        
        self.ui.visImu.append("<font color='green'>> Sensores funcionando asegurese que se puedan visualizar... </font>")

    def BUTTON_ON_pressed(self):
        if (self._FLAG_Habilitar_Robot==1):
            if self._FLAG_buttonOnPressed:
                logging.info("Boton ON ya presionado...")
                self.ui.visImu.append("<font color='black'>> Boton ON ya presionado... </font>")
                return
            """ Iniciamos el robot """
            # Habilitamos flag
            self._Thread_Robot = RobotReader(self)
            self._Thread_Robot.start()
            self._THREAD_socketReceptionData.start()
            self._FLAG_buttonOnPressed=True
            logging.info("Estableciendo conexion con el Robot")
            # Establecemos conexión con el socket y la transmisión de los comandos del joystick
            #self._THREAD_socketReceptionData.start()
            while not(self._FLAG_socketConected):
                logging.info("esperando conexión con el robot")
                time.sleep(1)
            logging.info("Conectado!")
            """ Configuramos camaras """
            # Define the codec and create VideoWriter object
            logging.info("Establecemos los VideoWriters")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use lower case
            frameRate=self._VAR_FRAME_RATE
            
            #nombreDirVid=os.path.join(MY_FILEPATH,"videos","video_TumiBot_cam1_{}.mp4".format(self._VAR_counterVideoFiles))
            #self._VAR_outVideoWrite_list[0] = cv2.VideoWriter(nombreDirVid, fourcc, frameRate, (self._VAR_WIDTH_FRAME_CAM, self._VAR_HEIGHT_FRAME_CAM))
            
            nombreDirVid=os.path.join(MY_FILEPATH,"videos","video_TumiBot_cam2_{}.mp4".format(self._VAR_counterVideoFiles))
            self._VAR_outVideoWrite_list[1] = cv2.VideoWriter(nombreDirVid, fourcc, frameRate, (self._VAR_WIDTH_FRAME_CAM, self._VAR_HEIGHT_FRAME_CAM))        
            self._THREAD_updateCamera1.start()
            self._THREAD_updateCamera2.start()
        else:
            self.ui.visImu.append("<font color='black'>>Espere, a una conexión con el ROBOT...</font>")
            self._FLAG_SENSORES_pressed = False
                
    def BUTTON_OFF_pressed(self):
        self.close()
        return
        logging.info("Cerrando conexión Robot")
        #Detenemos hilos de camara
        self._THREAD_updateCamera1.stop()
        self.ui.label_camMain.clear()
        self._THREAD_updateCamera2.stop()
        self.ui.label_camAruco.clear()
        logging.info("Se detuvo Cámaras")
        ## Detenemos hilo de Recepcion de datos
        self._THREAD_socketReceptionData.stop()
        logging.info("Se detuvo recepción de datos")
        # Habilitamos el comboBox
        self.ui.ComboBox_Seleccionador_Robot.setEnabled(True)
        """ Reiniciamos los hilos cerrados """
        # Crea el socket de comunicacion con el robot
        self._THREAD_socketReceptionData=socketReception(self)
        self._THREAD_socketReceptionData._SIGNAL_tramaDatos.connect(self.updateIndicators)
        self._THREAD_socketTranssmisionData=threading.Thread(target=self.socketTransmissionData,)
        # Camaras
        self._THREAD_updateCamera1=Camera(self,1)
        self._THREAD_updateCamera1.change_pixmap_signal.connect(self.update_image1)
        self._THREAD_updateCamera2=Camera(self,2)
        self._THREAD_updateCamera2.change_pixmap_signal.connect(self.update_image2)        
        # socket
        self._VAR_socketClient= None
        # joystick command
        self._VAR_joystickCommand=""
        # Video counter
        self._VAR_counterVideoFiles=0
        # List that contains the VideoWriter to MP4 of the video
        try:
            self._VAR_outVideoWrite_list[1].release()
            self._VAR_outVideoWrite_list[0].release()
        except:
            pass
        self._VAR_outVideoWrite_list=[None,None]
        logging.info("Desconectado!")
        self._FLAG_buttonOnPressed=False
          
    @pyqtSlot(np.ndarray)
    def update_image1(self, cv_img):
        """Updates the image_label with a new opencv image"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, width * channel, QImage.Format_RGB888)        
        scaledImage = q_image.scaled(self.ui.label_camMain.size(), aspectRatioMode=Qt.KeepAspectRatio)
        pixMap=QPixmap.fromImage(scaledImage)
        self.ui.label_camMain.setPixmap(pixMap)
        self.ui.label_camMain.setAlignment(Qt.AlignCenter)
    
    @pyqtSlot(np.ndarray)
    def update_image2(self, cv_img):
        """Updates the image_label with a new opencv image"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, width * channel, QImage.Format_RGB888)        
        scaledImage = q_image.scaled(self.ui.label_camAruco.size(), aspectRatioMode=Qt.KeepAspectRatio)
        pixMap=QPixmap.fromImage(scaledImage)
        self.ui.label_camAruco.setPixmap(pixMap)
        self.ui.label_camAruco.setAlignment(Qt.AlignCenter)
    
    def closeEvent(self, event):
        event.accept()
        logging.info("closing event")
        try:
            self._VAR_outVideoWrite_list[1].release()
            self._VAR_outVideoWrite_list[0].release()
        except:
            pass
        self._VAR_outVideoWrite_list=[None,None]
        time.sleep(1)
        os._exit(0)                 
def waitForLoading():
    global _FLAG_continue#,mesh_item
    t_start=time.time()
    """Creamos el directorio para videos """
    temp_videosFilePath=os.path.join(MY_FILEPATH,"videos")
    if not os.path.exists(temp_videosFilePath):
        ## Creamos el directorio
        os.mkdir(temp_videosFilePath)
        logging.info("Se ha creado el directorio: {}".format(temp_videosFilePath))
    else:
        logging.info("Ya existe el directorio: {}".format(temp_videosFilePath))
    
    #try:
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
    #except:
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
    while time.time()-t_start<2.5:
        pass
    _FLAG_continue=True

def setUpGUI():
    global _FLAG_continue
    ## Creamos la app
    app = QApplication(sys.argv)  
    # Cargar la imagen para el splash screen
    import pics_rc
    splash_image = QPixmap(":iconos/logotumi.png")
    splash = QSplashScreen(splash_image)
    splash.show()
    _FLAG_continue=False
    th=threading.Thread(target=waitForLoading,)
    th.start()
    while not(_FLAG_continue):
        pass
    splash.close()
    #Creamos el visualizador
    GUI_window=GUI()
    logging.info("______________INIT GUI____________")
    GUI_window.show()                        
    sys.exit(app.exec_())   
