# Library for PyQt
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys
from pygame.locals import *
import subprocess

# ROBOT INFO (RASPBERRY REMOTE)
ROBOT_TERRESTRE_IP_ADDR = "192.168.68.200"  # "192.168.1.166""192.168.1.133"
ROBOT_TERRESTRE_USERNAME = "pi"
ROBOT_TERRESTRE_PASSWORD = "raspberry"
puerto = 22  # Puerto por defecto de SSH
ROBOT_TERRESTRE_PORT = 8666
ROBOT_TERRESTRE_CAMS_PORT_LIST = [8080, 8085]
ROBOT_TERRESTRE_FRAME_RATE = 10
ROBOT_TERRESTRE_WIDTH_FRAME_CAM = 640
ROBOT_TERRESTRE_HEIGHT_FRAME_CAM = 480
ROBOT_TERRESTRE_MIN_VALOR_BATERIA = 22
ROBOT_TERRESTRE_RANGO_BATERIA = 25-22


class RobotNetworkCheckThread(QThread):
    # Señal para actualizar el estado de la conexión en la interfaz principal
    connection_status_signal = pyqtSignal(str)
    connection_status = pyqtSignal(int)

    def __init__(self, parent):
        super().__init__(None)
        self._FLAG_run = True
        self.falseParent = parent

    def run(self):
        value = 0
        while True:
            # Hacer ping a una dirección IP (puede ser a Google o una IP local en la red)
            ip = ROBOT_TERRESTRE_IP_ADDR        # Puedes poner una IP local si lo prefieres
            # Intentar hacer un ping

            try:
                # response = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

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
                        self.falseParent._Var_valueRobot += 1
                        if self.falseParent._Var_valueRobot == 1:
                            current_status = "<span style=\"font-family: 'MS Shell Dlg 2'; color: green; font-size: 17px;\">Robot CONECTADO!</span>"
                            self.connection_status_signal.emit(current_status)
                    else:
                        self.falseParent._Var_valueRobot = 0
                        self.connection_status.emit(0)
                        current_status = (
                            "<span style=\"font-family: 'MS Shell Dlg 2'; color: red; font-weight: bold; font-size: 17px;\">(ERROR)</span> "
                            "<span style=\"font-family: 'MS Shell Dlg 2'; color: white; font-size: 17px;\">Robot NO CONECTADO!</span>"
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
                        self.falseParent._Var_valueRobot += 1
                        if self.falseParent._Var_valueRobot == 1:
                            current_status = "<font color='green'>>Robot CONECTADO!</font>"
                            self.connection_status_signal.emit(current_status)
                    else:
                        self.falseParent._Var_valueRobot = 0
                        self.connection_status.emit(0)
                        current_status = "<font color='red'>>Robot NO CONECTADO!</font>"
                        self.connection_status_signal.emit(current_status)
                self.msleep(5000)
            except Exception as e:
                print(f"Error al hacer el ping: {e}")
