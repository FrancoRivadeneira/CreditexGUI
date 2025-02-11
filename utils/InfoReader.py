## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
from pygame.locals import *    ###
#import paramiko                ###

## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"

class InfoReader(QThread):
    _SIGNAL_data = pyqtSignal(list)
    connection_status_signal = pyqtSignal(str)
    
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        self.sshForInfo = paramiko.SSHClient()
        self.sshForInfo.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    def run(self):
        _VAR_counter=0
        logging.info("trying to stop the Save file Box")
        if True:
            try:
                self.sshForInfo.connect(LIDAR_MODULE_IP_ADDR, 
                            username=LIDAR_MODULE_USERNAME, 
                            password=LIDAR_MODULE_PASSWORD,
                            timeout=10)
                self.command = "source /opt/ros/melodic/setup.bash && rosnode info /livox_lidar_publisher2"
                stdin, stdout, stderr = self.sshForInfo.exec_command(self.command, get_pty=True)
                output = stdout.read().decode()
                error = stderr.read().decode()
                filtered_lines = [
                    line[:line.find('[')].strip() if '[' in line and 'Node' not in line else line.strip()  # Elimina contenido entre corchetes, excepto si la línea tiene 'Node'
                    for line in output.splitlines()
                    if '* to:' not in line 
                    and '* direction:' not in line 
                    and '* transport:' not in line 
                    and line.strip() != '' 
                    and not line.startswith('Services:') 
                    and not line.startswith(' * /main_node/get_loggers') 
                    and not line.startswith(' * /main_node/set_logger_level') 
                    and not line.startswith('contacting node')
                    and not line.startswith(' * topic: /rosout')
                    and not line.startswith(' * /rosout [rosgraph_msgs/Log]')
                    and not line.startswith('------------------------------')
                ]
                output = "\n".join(filtered_lines)
                output = "\n----------\n"+output +"\n----------"
                output1 = output
                #self.connection_status_signal1.emit(output)
                self.command = "source /opt/ros/melodic/setup.bash && rosnode info /main_node "
                stdin, stdout, stderr = self.sshForInfo.exec_command(self.command, get_pty=True)
                output = stdout.read().decode()
                error = stderr.read().decode()
                filtered_lines = [
                    line[:line.find('[')].strip() if '[' in line and 'Node' not in line else line.strip()  # Elimina contenido entre corchetes, excepto si la línea tiene 'Node'
                    for line in output.splitlines()
                    if '* to:' not in line 
                    and '* direction:' not in line 
                    and '* transport:' not in line 
                    and line.strip() != '' 
                    and not line.startswith('Services:') 
                    and not line.startswith(' * /main_node/get_loggers') 
                    and not line.startswith(' * /main_node/set_logger_level') 
                    and not line.startswith('contacting node')
                    and not line.startswith(' * topic: /rosout')
                    and not line.startswith(' * /rosout [rosgraph_msgs/Log]')
                    and not line.startswith('------------------------------')
                ]

                

                output = "\n".join(filtered_lines)
                output = output +"\n----------"
                output2 = output
                #self.connection_status_signal2.emit(output)
                self.command = "ls -lh /home/jetson/external/grabacion/*.bag | awk '{print $5, $9}'"
                stdin, stdout, stderr = self.sshForInfo.exec_command(self.command, get_pty=True)
                output = stdout.read().decode()
                error = stderr.read().decode()
                output = output.replace("/home/jetson/external/grabacion/","")
                output3 = output
                output = output1+ "\n" + output2 + "\n" +output3
                self.connection_status_signal.emit(output)

                _VAR_counter = 1
            except Exception as e:
                logging.error(f"Error en la conexión SSH: {e}")
                self.connection_status_signal.emit("<font color='red'>>Hubo un problema con la conexion a ethernet intente volver a conectar...</font>")
                #self.finished.emit(f"Error: {e}")
            finally:
                self.sshForInfo.close()
