## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging 
from pygame.locals import *    ###
#import paramiko
                ###
## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"

class StopReader(QThread):
    _SIGNAL_data = pyqtSignal(list)
    connection_status_signal = pyqtSignal(str)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        self.sshForStop = paramiko.SSHClient()
        self.sshForStop.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    def run(self):
        _VAR_counter=0
        logging.info("trying to stop the Save file Box")
        if True:
            try:
                self.sshForStop.connect(LIDAR_MODULE_IP_ADDR, 
                        username=LIDAR_MODULE_USERNAME, 
                        password=LIDAR_MODULE_PASSWORD,
                        timeout=10)
                #self.command = "bash /home/jetson/external/grabacion/detener.bash"
                self.command = "bash /home/jetson/Grabar_en_USB/detener.bash"
                stdin, stdout, stderr = self.sshForStop.exec_command(self.command, get_pty=True)
                output = stdout.read().decode()
                error = stderr.read().decode()
                logging.info(f"Output: {output}")
                #print(type(output))
                if error:
                    logging.error(f"Error al ejecutar el comando: {error}")
                else:
                    logging.info("Comando ejecutado correctamente.")
                logging.info("Conexion por ssh establecida para la grabacion. ")
                #self.command = "ls -lh /home/jetson/external/grabacion/ -- *.bag | awk '{print $5, $9}'"
                #stdin, stdout, stderr = self.sshForStop.exec_command(self.command, get_pty=True)
                #output = stdout.read().decode()
                #error = stderr.read().decode()
                self.connection_status_signal.emit("<font color='green'>>Se detuvo la grabacion de los archivos de video y bag, se recomienda usar el boton info para ver los estados de sus archivos...</font>")
                self.falseParent._sshStopConnected=True
                _VAR_counter = 1
            except Exception as e:
                logging.error(f"Error en la conexión SSH: {e}")
                self.connection_status_signal.emit("<font color='red'>>Hubo un problema con la conexion a ethernet intente volver a conectar...</font>")
            finally:
                self.sshForStop.close()
