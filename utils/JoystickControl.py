## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
import logging ## Libreria para info Logging
import time
import pygame                  ###
from pygame.locals import *    ###
import sys

actuator_pos=0
""" COMANDOS PARA LA NUCLEO"""

comandoList=[ ["$OAX3J0A",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0],"Avanzar"],
              ["$OAX3J0B",[0,1,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0],"Retroceder"],
              ["$OAX3J0r",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1,0,0],"Giro Horario"],
              ["$OAX3J0l",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 1,0,0,0],"Giro Antihorario"],
              ["$OAX3JR1",[0,0,0,0,1, 0,0,0,0,0, 0,1,0,0,0, 0,0,0,0],"Prender Luces"],
              ["$OAX3JR2",[0,0,0,0,1, 0,0,0,0,0, 0,0,1,0,0, 0,0,0,0],"Apagar Luces"],
              ["$OAX3JX1",[0,0,1,0,0, 0,0,0,0,0, 0,0,0,1,0, 0,0,0,0],"Aumentar Velocidad"],
              ["$OAX3JX2",[0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,1, 0,0,0,0],"Disminuir Velocidad"],              
              ["$OAX3JRA",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1,0,0],"Avanzar Derecha"],
              ["$OAX3JLA",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 1,0,0,0],"Avanzar Izquierda"],
              ["$OAX1M"  ,[0,0,0,1,0, 0,0,0,0,0, 0,0,0,1,0, 0,0,0,0],"Activar Motores"], ### CAMBIAR ESTO es Y + FlechaArriba
              ["$OAX3JAR",[0,0,0,1,0, 0,0,0,0,0, 0,0,0,0,1, 0,0,0,0],"Apagar Rele"], ## CAMBIAR ESTO es Y + FlechaAbajo
              ["$OAX3JBE",[0,0,0,1,0, 0,0,0,0,0, 0,0,1,0,0, 0,0,0,0],"Boton Emergencia"],
              ["$OAX2R1",[0,0,0,0,0, 0,0,0,0,1, 0,0,0,0,0, 0,0,0,0],"Activar Actuador"],
              ["$OAX2R0",[0,0,0,0,0, 0,0,0,1,0, 0,0,0,0,0, 0,0,0,0],"Desactivar Actuador"],
              ["$OAX3JLU",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,1,0],"Subir Actuador"],
              ["$OAX3JLD",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,1],"Bajar Actuador"]] ## CAMBIAR ESTO es Y + FlechaIzquierda

## Aplicamos transpuesta a la lista de comandos para hacer uso del built function "index"
comandoList = [list(x) for x in zip(*comandoList)]

EVENT_TYPE_LEFT_TRIGGER=4
if sys.platform=="linux":
    EVENT_TYPE_LEFT_TRIGGER=2


class JoystickControl(QThread):
    _SIGNAL_command=pyqtSignal(str)
    _SIGNAL_updateJoystickIndicator=pyqtSignal(int)
    def __init__(self,parent=None):
        super().__init__(None)
        self._run_flag = True
        self.falseParent=parent
    
        
        

    def run(self):
        pygame.init()
        pygame.joystick.init()
        clock=pygame.time.Clock()
        """
        -----------------------BUTTONS-------------------------
        boton_A ID:0
        boton_B ID:1
        boton_X ID:2
        boton_Y ID:3
        boton_LB ID:4
        boton_RB ID:5
        boton_back ID:6
        boton_start ID:7
        boton_logitech ID:8
        boton_LeftStick ID:9
        boton_RightStick ID:10
        """
        ##--------------------------PAD---------------------------
        idFlecha_Derecha=11
        idFlecha_Izquierda=12
        idFlecha_Arriba=13
        idFlecha_Abajo=14
        ##----------------------TRIGGER---------------------------
        idTrigger_Left=15
        idTrigger_Right=16
        ## Creamos el registro que contendrá el código indicando q botones se han presionado
        buttonList=[0 for i in range(19)]
        _FLAG_JoystickConnected=False
        oldCommand=""
        while self._run_flag:
            try:
                """ Verificamos que hay un joystick conectado """
                if pygame.joystick.get_count() == 0:
                    _FLAG_JoystickConnected=False
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    self._SIGNAL_updateJoystickIndicator.emit(0)
                    time.sleep(0.5)
                    continue
                """ Inicializamos un Joystick """
                if not(_FLAG_JoystickConnected):
                    ## Realizamos la conexión del joystick
                    joysticks=[]
                    for i in range(pygame.joystick.get_count()):    
                        joysticks.append(pygame.joystick.Joystick(i))
                        joysticks[-1].init() #Inicializa el joystick
                    self._SIGNAL_updateJoystickIndicator.emit(100)
                    _FLAG_JoystickConnected=True
                    logging.info("Configuración del joystick terminada")
                """ Se detecta los botones presionados """
                for event in pygame.event.get():
                    if event.type == JOYBUTTONDOWN: #Si se presiona un boton
                        #Se analiza que botón se presionó. Cada boton tiene un número específico.
                        buttonList[event.button]=1
                    if event.type == JOYBUTTONUP:#Si se suelta un boton
                        #Se analiza que botón se soltó
                        buttonList[event.button]=0

                    if event.type == JOYHATMOTION: #Si se presiona una flecha. Este evento es un array 2x1, guarda el estado de las flechas arriba/abajo y derecha/izquierda
                        if event.value[0] ==1:
                            buttonList[idFlecha_Derecha]=1
                            
                        if event.value[0] ==-1:
                            buttonList[idFlecha_Izquierda]=1
                            
                        if event.value[0] ==0:
                            buttonList[idFlecha_Derecha]=0
                            buttonList[idFlecha_Izquierda]=0
                            
                        if event.value[1] ==1:
                            buttonList[idFlecha_Arriba]=1
                            
                        if event.value[1] ==-1:
                            buttonList[idFlecha_Abajo]=1
                            
                        if event.value[1] ==0:
                            buttonList[idFlecha_Arriba]=0
                            buttonList[idFlecha_Abajo]=0
                    if event.type == JOYAXISMOTION:#Si se presiona una trigger (RT o LT)
                        #En el caso de los triggers, se analiza que tanto se ha presionado
                        #va de un rango de -0.99 (no presionado) a 0.99 (totalmente presionado)
                        global actuator_pos
                        if event.axis == EVENT_TYPE_LEFT_TRIGGER:
                            if (event.value <0.99):
                                buttonList[idTrigger_Left]=0
                            else:
                                buttonList[idTrigger_Left]=1
                        if event.axis == 5:
                            if (event.value <0.99):
                                buttonList[idTrigger_Right]=0 #Se actualiza la variable correspondiente
                            else:
                                buttonList[idTrigger_Right]=1
                        # if event.axis == 3:
                            
                        #     # Ajustar el contador según la inclinación del joystick
                        #     if event.value < -0.5:  # Movimiento hacia arriba
                        #         actuator_pos += 1
                        #         buttonList[17]=1
                        #         buttonList[18]=0
                                                      
                        #     elif event.value > 0.5:  # Movimiento hacia abajo
                        #         actuator_pos -= 1
                        #         buttonList[17]=0
                        #         buttonList[18]=1
                        #     else:
                        #         buttonList[17]=0
                        #         buttonList[18]=0
                            # global comandoList
                            # comandoList[0][13] = f"$OAX4G{str(actuator_pos).zfill(3)}"
                            # print(comandoList[13][0])
                                                            
                ## Comparamos el registro
                # Verificar si hay al menos un joystick conectado
                    Estado_conexion = 0
                    ## No existira comando, actualizamos valor a enviar
                """ Enviamos el comando por el socket """
                # print(buttonList)
                # print(comandoList[0][13])
                if buttonList in comandoList[1]:
                    id=comandoList[1].index(buttonList)
                    ## Obtenemos el comando
                    command=comandoList[0][id]
                    if oldCommand != command:
                        logging.info("Comando {} enviado ({})".format(comandoList[2][id],command))
                    oldCommand=command
                    self._SIGNAL_command.emit(command)
                    time.sleep(0.02)
                clock.tick(200) ## Genera delay constante
            except KeyboardInterrupt:
                logging.info("cerrando bucle")
                break
            except:
                logging.info("error detectando comando")
                time.sleep(2)
