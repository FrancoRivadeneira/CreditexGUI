## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
import time
from pygame.locals import *    ###
# #import paramiko                ###

## LIDAR Module Info
LIDAR_MODULE_IP_ADDR= "192.168.1.4"  #"192.168.137.4"
LIDAR_MODULE_USERNAME= "jetson"          #"pi" #"nvidia"
LIDAR_MODULE_PASSWORD = "jetson"  #"raspberry" #"nvidia"

class EncoderDataReader(QThread):
    _SIGNAL_data = pyqtSignal(str)
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
        
    def run(self): ## Hilo
        cont = 0
        conectado = False
        while self._FLAG_run:
            try:
                if not(self.falseParent._FLAG_sshEncoderConnected):
                    logging.info("trying to create ssh /robot/encoder")
                    # sshForEncoder = paramiko.SSHClient()
                    # sshForEncoder.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    # sshForEncoder.connect(LIDAR_MODULE_IP_ADDR, username=LIDAR_MODULE_USERNAME, password=LIDAR_MODULE_PASSWORD)
                    self.falseParent._FLAG_sshEncoderConnected=True
                    
                    ros_setup_script = '/opt/ros/melodic/setup.bash'
                    
                    command=f"source {ros_setup_script} && rostopic echo /robot/encoder"
                    
                    #command=f"rostopic echo /scan -w 4"
                    
                    # stdin, stdout, stderr = sshForEncoder.exec_command(command)
                    #time.sleep(0.5)
                    logging.info("ssh for /robot/encoder topic created")
                    timeoutCounter=time.time()
                    tiempo_Contador = time.time()

                # data=stdout.readline()

                if len(data)==0 or data[0:4] != "data":
                    if time.time()-timeoutCounter>5:
                        # No hay data, debemos verificar que se envíe el comando
                        logging.info("No hay data transmitida para el Encoder")
                        timeoutCounter=time.time()
                        try:
                            # sshForEncoder.close()
                            pass
                        except:
                            pass
                        self.falseParent._FLAG_sshEncoderConnected=False
                    continue


                if data[0:4] == "data":
                    #print(data[6:])
                    data = data[6:]
                    self._SIGNAL_data.emit(data)
                    cont += 1
                    conectado = True

                    tiempo_Contador = time.time()
                    timeoutCounter=time.time()
                    
                    

                    if cont % 2 == 0:
                        self.falseParent._FLAG_EncoderFuncionando=True
                    else:
                        self.falseParent._FLAG_EncoderFuncionando=False
                else:
                    conectado = False

                
                

                timeoutCounter=time.time()


            except KeyboardInterrupt:
                exit()                
            except:
                self.falseParent._FLAG_sshEncoderConnected=False
                logging.info("Error with ssh Lidar")
                continue
            
    def stop(self):
        self._FLAG_run=False
        self.wait()  
