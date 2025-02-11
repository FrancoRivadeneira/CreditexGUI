## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
from pygame.locals import *    ###
import paramiko                ###
## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"
class CajaSensorReader(QThread):
    _SIGNAL_data = pyqtSignal(list)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        self.sshForSensors = paramiko.SSHClient()
        self.sshForSensors.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    def run(self):
        _VAR_counter=0
        logging.info("trying to communicate with the Sensors Box")
        if True:
            try:
                self.sshForSensors.connect(LIDAR_MODULE_IP_ADDR, 
                        username=LIDAR_MODULE_USERNAME, 
                        password=LIDAR_MODULE_PASSWORD,
                        timeout=10)
                self.command = "bash /home/jetson/teleoperados_WS/ejecucion.bash"
                stdin, stdout, stderr = self.sshForSensors.exec_command(self.command, get_pty=True)
                output = stdout.read().decode()
                error = stderr.read().decode()
                logging.info(f"Output: {output}")
                if error:
                    logging.error(f"Error al ejecutar el comando: {error}")
                else:
                    logging.info("Comando ejecutado correctamente.")
                logging.info(f"Ejecutando comando: {self.command}")
                logging.info("Conexion por ssh establecida para la caja de sensores")
                self.falseParent._sshCajaSensoresConnected=True
                _VAR_counter = 1
                #time.sleep(60)
            except Exception as e:
                logging.error(f"Error en la conexión SSH: {e}")
                self.finished.emit(f"Error: {e}")
            finally:
                self.sshForSensors.close()
