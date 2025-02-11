## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging           ###
import time
from pygame.locals import *    ###
#import paramiko                ###

## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"

class lidarDataReader(QThread):
    _SIGNAL_data = pyqtSignal(str)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        
    def run(self): ## Hilo
        cont = 0
        while self._FLAG_run:
            try:
                if not(self.falseParent._FLAG_sshLidarConnected):
                    logging.info("trying to create ssh /scan")
                    sshForLidar = paramiko.SSHClient()
                    sshForLidar.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    sshForLidar.connect(LIDAR_MODULE_IP_ADDR, username=LIDAR_MODULE_USERNAME, password=LIDAR_MODULE_PASSWORD)
                    self.falseParent._FLAG_sshLidarConnected=True
                    
                    ros_setup_script = '/opt/ros/melodic/setup.bash'
                    
                    command=f"source {ros_setup_script} && rostopic echo /scan"
                    
                    #command=f"rostopic echo /scan -w 4"
                    
                    stdin, stdout, stderr = sshForLidar.exec_command(command)
                    #time.sleep(0.5)
                    logging.info("ssh for /scan topic created")
                    timeoutCounter=time.time()
                data=stdout.readline()

                if data[0:6] == "ranges":
                    data = data[8:]
                    self._SIGNAL_data.emit(data)
                    cont += 1
                    timeoutCounter=time.time()
                    if cont % 2 == 0:
                        self.falseParent._FLAG_LidarFuncionando = True
                    else:
                        self.falseParent._FLAG_LidarFuncionando = False

                if len(data)==0 or data[0:6] != "ranges":
                    if time.time() - timeoutCounter>5:
                        # No hay data, debemos verificar que se envíe el comando
                        logging.info("No hay data transmitida para el Lidar")
                        timeoutCounter=time.time()
                        try:
                            sshForLidar.close()
                        except:
                            pass
                        self.falseParent._FLAG_sshLidarConnected=False
                    continue


                

                #self._SIGNAL_data.emit(data)


            except KeyboardInterrupt:
                exit()                
            except:
                self.falseParent._FLAG_sshLidarConnected=False
                logging.info("Error with ssh Lidar")
                continue
            
    def stop(self):
        self._FLAG_run=False
        self.wait()
