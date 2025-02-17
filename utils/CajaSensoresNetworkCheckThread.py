## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
## For Camera visualization  ###
import sys               ###
from pygame.locals import *    ###
import subprocess

## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"



class CajaSensoresNetworkCheckThread(QThread):
    connection_status_signal = pyqtSignal(str)
    connection_status = pyqtSignal(int)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent

    def run(self):
        value = 0
        while True:
            # Hacer ping a una dirección IP (puede ser a Google o una IP local en la red)
            ip = LIDAR_MODULE_IP_ADDR        # Puedes poner una IP local si lo prefieres
            #response = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                # Intentar hacer un ping
                # Verificar si el ping fue exitoso
                if sys.platform == "win32":
                    command = ["ping", "-n", "1", ip]
                    process = subprocess.Popen(command, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    stdin=subprocess.PIPE,
                    creationflags=subprocess.CREATE_NO_WINDOW)
                    stdout, stderr = process.communicate()
                    # En Windows, el ping exitoso muestra algo como "bytes=32 tiempo=xxms TTL=xx"
                    if b"bytes=" in stdout:
                        self.connection_status.emit(1)
                        self.falseParent._Var_valueCajaSensores += 1
                        if self.falseParent._Var_valueCajaSensores == 1:
                            current_status = "<font face='MS Shell Dlg 2' color='green'>Caja sensores CONECTADO!</font>"
                            self.connection_status_signal.emit(current_status)
                    else:
                        self.falseParent._Var_valueCajaSensores = 0
                        self.connection_status.emit(0)
                        current_status = (
                            "<span style=\"font-family: 'MS Shell Dlg 2'; color: red; font-weight: bold; font-size: 17px;\">(ERROR)</span> "
                            "<span style=\"font-family: 'MS Shell Dlg 2'; color: white; font-size: 17px;\">Caja Sensores NO CONECTADO!</span>"
                        )

                        self.connection_status_signal.emit(current_status)
                else:
                    command = ["ping", "-c", "1", ip]
                    process = subprocess.Popen(command, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    stdin=subprocess.PIPE)
                    stdout, stderr = process.communicate()
                    # En Linux, el ping exitoso muestra algo como "bytes from x.x.x.x: icmp_seq=1 ttl=64 time=xx ms"
                    if b"bytes from" in stdout:
                        self.connection_status.emit(1)
                        self.falseParent._Var_valueCajaSensores += 1
                        if self.falseParent._Var_valueCajaSensores == 1:
                            current_status = "<font color='green'>>Caja sensores CONECTADO!</font>"
                            self.connection_status_signal.emit(current_status)
                    else:
                        self.falseParent._Var_valueCajaSensores = 0
                        self.connection_status.emit(0)
                        current_status = "<font color='red'>>Caja sensores NO CONECTADO!</font>"
                        self.connection_status_signal.emit(current_status)   
                
                # Esperar 5 segundos antes de realizar el siguiente ping
                self.msleep(5000)
            except Exception as e:
                print(f"Error al hacer el ping: {e}")
