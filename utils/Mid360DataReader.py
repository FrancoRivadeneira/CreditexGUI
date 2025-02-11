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


class Mid360DataReader(QThread):
    _SIGNAL_data = pyqtSignal(str)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        
    def run(self): ## Hilo
        cont = 0
        cont_0 = 0
        while self._FLAG_run:
            try:
                if not(self.falseParent._FLAG_sshMID360Connected):
                    logging.info("trying to create ssh /livox/lidar")
                    sshForMID360 = paramiko.SSHClient()
                    sshForMID360.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    sshForMID360.connect(LIDAR_MODULE_IP_ADDR, username=LIDAR_MODULE_USERNAME, password=LIDAR_MODULE_PASSWORD)
                    self.falseParent._FLAG_sshMID360Connected=True
                    ros_setup_script = '/opt/ros/melodic/setup.bash'
                    command=f"source {ros_setup_script} && rostopic echo /numCloud"
                    
                    stdin, stdout, stderr = sshForMID360.exec_command(command)
                    #time.sleep(0.5)
                    logging.info("ssh for /livox/lidar topic created")
                    timeoutCounter=time.time()
                data=stdout.readline()
                #cont_0 +=1
                
                if data[0:4]  == "data":
                    #print(len(data))
                    data = data
                    #print(data[0:4])
                    self._SIGNAL_data.emit(data)
                    cont += 1
                    timeoutCounter=time.time()
                    #time.sleep(0.5)
                    if cont % 2 == 0:
                        self.falseParent._FLAG_MID360Funcionando = True
                    else:
                        self.falseParent._FLAG_MID360Funcionando = False
                        
                        
                #if len(data)==0 or data[0:4] != "data":
                #    if time.time() - timeoutCounter>5:
                #        # No hay data, debemos verificar que se envíe el comando
                #        logging.info("No hay data transmitida para el livox Lidar")
                #        timeoutCounter=time.time()
                #        try:
                #            sshForMID360.close()
                #        except:
                #            pass
                #        self.falseParent._FLAG_sshLidarConnected=False
                #    continue
                
                
            except KeyboardInterrupt:
                exit()                
            except:
                self.falseParent._FLAG_sshMID360Connected=False
                logging.info("Error with ssh Lidar")
                continue
            
    def stop(self):
        self._FLAG_run=False
        self.wait()
