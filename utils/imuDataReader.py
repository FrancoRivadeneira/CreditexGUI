## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
import time
from pygame.locals import *    ###
#import paramiko                ###

## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"

class imuDataReader(QThread):
    _SIGNAL_data = pyqtSignal(list)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        
    def run(self): ## Hilo
        while self._FLAG_run:
            try:
                if not(self.falseParent._FLAG_sshImuConnected):
                    logging.info("trying to create ssh /imu/data_euler")
                    sshForImu = paramiko.SSHClient()
                    sshForImu.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    sshForImu.connect(LIDAR_MODULE_IP_ADDR, username=LIDAR_MODULE_USERNAME, password=LIDAR_MODULE_PASSWORD)
                    logging.info("Conexion por ssh establecida para el IMU")
                    self.falseParent._FLAG_sshImuConnected=True
                    ros_setup_script = '/opt/ros/melodic/setup.bash'
                    command=f"source {ros_setup_script} && rostopic echo /imu/data_euler -w7"
                    stdin, stdout, stderr = sshForImu.exec_command(command,get_pty=True)
                    logging.info("ssh for /imu topic created")
                    #time.sleep(0.5)
                    # Reiniciamos las variables
                    data=[]
                    timeoutCounter=time.time()
                # Transformamos la data en tipo float
                line=stdout.readline()
                #print("LOL")
                #print(line.strip())
                if len(line)==0 or line[:2]!="x:":
                    if time.time()-timeoutCounter>5:
                        # No hay data, debemos verificar que se envíe el comando
                        logging.info("No hay data transmitida para el Imu")
                        timeoutCounter=time.time()
                        try:
                            sshForImu.close()
                        except:
                            pass
                        self.falseParent._FLAG_sshImuConnected=False
                    continue
                timeoutCounter=time.time()
                data.append(float(line[2:]))
                counter=1
                while counter<3:
                    line=stdout.readline()
                    if len(line)==0 or line[1]!=":":
                        continue
                    data.append(float(line[2:]))
                    counter+=1
                self._SIGNAL_data.emit(data)
                #print(data)
                data=[]
            except KeyboardInterrupt:
                exit()
            except:
                self.falseParent._FLAG_sshImuConnected=False
                logging.info("error with ssh IMU")
                continue
    
    def stop(self):
        self._FLAG_run=False
        self.wait()  
