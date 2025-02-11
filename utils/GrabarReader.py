## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
from pygame.locals import *    ###
# #import paramiko                ###


## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"

class GrabarReader(QThread):
    _SIGNAL_data = pyqtSignal(list)
    connection_status_signal = pyqtSignal(str)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        # self.sshForGrabacion = paramiko.SSHClient()
        # self.sshForGrabacion.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    def run(self):
        _VAR_counter=0
        logging.info("trying to communicate with the Save file Box")
        if True:
            try:
                #print(self.falseParent._VAR_Nombre_GRabacion)
                self.sshForGrabacion.connect(LIDAR_MODULE_IP_ADDR, 
                        username=LIDAR_MODULE_USERNAME, 
                        password=LIDAR_MODULE_PASSWORD,
                        timeout=10)
                
                self.connection_status_signal.emit("<font color='black'>>El proceso de grabación de los archivos de video y BAG ha comenzado. Se recomienda utilizar el botón 'Info' para consultar el estado de los archivos.</font>")
                #self.command = "bash /home/jetson/external/grabacion/record.bash"
                self.command = "bash /home/jetson/Grabar_en_USB/record.bash" + " " + self.falseParent._VAR_Nombre_GRabacion
                stdin, stdout, stderr = self.sshForGrabacion.exec_command(self.command, get_pty=True)
                output = stdout.read().decode()
                error = stderr.read().decode()
                logging.info(f"Output: {output}")
                if error:
                    logging.error(f"Error al ejecutar el comando: {error}")
                else:
                    logging.info("Comando ejecutado correctamente.")
                logging.info("Conexion por ssh establecida para la grabacion. ")
                
                self.falseParent._sshGrabacionConnected=True
                _VAR_counter = 1
            except Exception as e:
                logging.error(f"Error en la conexión SSH: {e}")
                self.connection_status_signal.emit("<font color='red'>>Hubo un problema con la conexion a ethernet intente volver a conectar...</font>")
                #self.finished.emit(f"Error: {e}")
            finally:
                self.sshForGrabacion.close()
        else:
            logging.info("No esta conectado a la caja de sensores")
