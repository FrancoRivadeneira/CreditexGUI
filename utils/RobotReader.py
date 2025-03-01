# Library for PyQt
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import logging  # Libreria para info Logging
from pygame.locals import *
import paramiko
ROBOT_TERRESTRE_IP_ADDR = "192.168.68.200"  # "192.168.1.166""192.168.1.133"
ROBOT_TERRESTRE_USERNAME = "pi"
ROBOT_TERRESTRE_PASSWORD = "raspberry"


class RobotReader(QThread):
    _SIGNAL_data = pyqtSignal(list)

    def __init__(self, parent):
        super().__init__(None)
        self._FLAG_run = True
        self.falseParent = parent
        self.sshForRobot = paramiko.SSHClient()
        self.sshForRobot.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    def run(self):
        _VAR_counter = 0
        logging.info("trying to communicate with the robot")
        if True:
            try:
                self.sshForRobot.connect(ROBOT_TERRESTRE_IP_ADDR,
                                         username=ROBOT_TERRESTRE_USERNAME,
                                         password=ROBOT_TERRESTRE_PASSWORD,
                                         timeout=10)
                self.command = "bash /home/pi/robot_ejecucion/remoteRobot.sh"
                stdin, stdout, stderr = self.sshForRobot.exec_command(
                    self.command, get_pty=True)
                output = stdout.read().decode()
                error = stderr.read().decode()
                if error:
                    logging.error(f"Error al ejecutar el comando: {error}")

                else:
                    logging.info("Comando ejecutado correctamente.")
                logging.info(f"Ejecutando comando: {self.command}")
                logging.info("Conexion por ssh establecida para el ROBOT")
                self.falseParent._sshRobotConnected = True
                _VAR_counter = 1
            except Exception as e:
                logging.error(f"Error en la conexión SSH: {e}")
                self.finished.emit(f"Error: {e}")
            finally:
                self.sshForRobot.close()
