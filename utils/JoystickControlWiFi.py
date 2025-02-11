import pygame
import time
import threading
from pygame.locals import *
import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *


""" COMANDOS PARA LA NUCLEO"""
                                          #A, B, X, Y, LB RB,back,start,LJ,RJ  -,der,izq,ar,ab  rt,rl
# ROBOT_LIST_COMMANDS = [["$OAX9X00500050", [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], "Avanzar"],
#                        ["$OAX9X10501050", [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], "Retroceder"],
#                        ['$OAX9X10500050', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], "Giro Horario"],
#                        ["$OAX9X00501050", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], "Giro Antihorario"],
#                        ["$OAX1Y",         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], "Prender Luces"],
#                        ["$OAX1Y",         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], "Apagar Luces"],
#                        ["$OAX3JX1",       [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 1, 0, 0, 0], "Aumentar Velocidad"],
#                        ["$OAX3JX2",       [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], "Disminuir Velocidad"],
#                        ["$OAX1M",         [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],"Activar Motores"], 
#                        ["$OAX3JLA",       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], "Boton atras"],
#                        ["$OAX3JAR",       [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], "Apagar Rele"],
#                        ["$OAX3JBE",       [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], "Boton de emergencia"],
#                     ]
""" Robot de Chile"""
ROBOT_LIST_COMMANDS=[ ["$OAX3J0A",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0],"Avanzar"],
              ["$OAX3J0B",[0,1,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0],"Retroceder"],
              ["$OAX3J0r",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1],"Giro Horario"],
              ["$OAX3J0l",[0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 1,0],"Giro Antihorario"],
              ["$OAX3JR1",[0,0,0,0,1, 0,0,0,0,0, 0,1,0,0,0, 0,0],"Prender Luces"],
              ["$OAX3JR2",[0,0,0,0,1, 0,0,0,0,0, 0,0,1,0,0, 0,0],"Apagar Luces"],
              ["$OAX3JX1",[0,0,1,0,0, 0,0,0,0,0, 0,0,0,1,0, 0,0],"Aumentar Velocidad"],
              ["$OAX3JX2",[0,0,1,0,0, 0,0,0,0,0, 0,0,0,0,1, 0,0],"Disminuir Velocidad"],              
              ["$OAX3JRA",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1],"Avanzar Derecha"],
              ["$OAX3JLA",[1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 1,0],"Avanzar Izquierda"],
              ["$OAX1M"  ,[0,0,0,1,0, 0,0,0,0,0, 0,0,0,1,0, 0,0],"Activar Motores"], ### CAMBIAR ESTO es Y + FlechaArriba
              ["$OAX3JAR",[0,0,0,1,0, 0,0,0,0,0, 0,0,0,0,1, 0,0],"Apagar Rele"], ## CAMBIAR ESTO es Y + FlechaAbajo
              ["$OAX3JBE",[0,0,0,1,0, 0,0,0,0,0, 0,0,1,0,0, 0,0],"Boton Emergencia"]] ## CAMBIAR ESTO es Y + FlechaIzquierda
# Aplicamos transpuesta a la lista de comandos para hacer uso del built function "index"
ROBOT_LIST_COMMANDS = [list(x) for x in zip(*ROBOT_LIST_COMMANDS)]
# joystick
EVENT_TYPE_LEFT_TRIGGER = 4
if sys.platform == "linux":
    EVENT_TYPE_LEFT_TRIGGER = 2

exit_flag = threading.Event()
class JoystickControlWiFi(QThread):
    global _VAR_joystickCommand
    _SIGNAL_updateJoystickIndicator = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(None)
        self.falseParent = parent

    def run(self):
        pygame.init()
        pygame.joystick.init()
        clock = pygame.time.Clock()
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
        # --------------------------PAD---------------------------
        idFlecha_Derecha = 11
        idFlecha_Izquierda = 12
        idFlecha_Arriba = 13
        idFlecha_Abajo = 14
        # ----------------------TRIGGER---------------------------
        idTrigger_Left = 15
        idTrigger_Right = 16
        # Creamos el registro que contendrá el código indicando q botones se han presionado
        buttonList = [0 for i in range(17)]
        _FLAG_JoystickConnected = False
        oldCommand = ""
        while not exit_flag.is_set():
            try:
                """ Verificamos que hay un joystick conectado """
                if pygame.joystick.get_count() == 0:
                    _FLAG_JoystickConnected = False
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    self._SIGNAL_updateJoystickIndicator.emit(0)
                    time.sleep(0.5)
                    continue
                """ Inicializamos un Joystick """
                if not (_FLAG_JoystickConnected):
                    # Realizamos la conexión del joystick
                    joysticks = []
                    for i in range(pygame.joystick.get_count()):
                        joysticks.append(pygame.joystick.Joystick(i))
                        joysticks[-1].init()  # Inicializa el joystick
                    self._SIGNAL_updateJoystickIndicator.emit(100)
                    _FLAG_JoystickConnected = True
                    print("Configuración del joystick terminada")
                """ Se detecta los botones presionados """
                for event in pygame.event.get():
                    if event.type == JOYBUTTONDOWN:  # Si se presiona un boton
                        # Se analiza que botón se presionó. Cada boton tiene un número específico.
                        buttonList[event.button] = 1
                    if event.type == JOYBUTTONUP:  # Si se suelta un boton
                        # Se analiza que botón se soltó
                        buttonList[event.button] = 0

                    if event.type == JOYHATMOTION:  # Si se presiona una flecha. Este evento es un array 2x1, guarda el estado de las flechas arriba/abajo y derecha/izquierda
                        if event.value[0] == 1:
                            buttonList[idFlecha_Derecha] = 1

                        if event.value[0] == -1:
                            buttonList[idFlecha_Izquierda] = 1

                        if event.value[0] == 0:
                            buttonList[idFlecha_Derecha] = 0
                            buttonList[idFlecha_Izquierda] = 0

                        if event.value[1] == 1:
                            buttonList[idFlecha_Arriba] = 1

                        if event.value[1] == -1:
                            buttonList[idFlecha_Abajo] = 1

                        if event.value[1] == 0:
                            buttonList[idFlecha_Arriba] = 0
                            buttonList[idFlecha_Abajo] = 0
                    # Si se presiona una trigger (RT o LT)
                    if event.type == JOYAXISMOTION:
                        # En el caso de los triggers, se analiza que tanto se ha presionado
                        # va de un rango de -0.99 (no presionado) a 0.99 (totalmente presionado)
                        if event.axis == EVENT_TYPE_LEFT_TRIGGER:
                            if (event.value < 0.99):
                                buttonList[idTrigger_Left] = 0
                            else:
                                buttonList[idTrigger_Left] = 1
                        if event.axis == 5:
                            if (event.value < 0.99):
                                # Se actualiza la variable correspondiente
                                buttonList[idTrigger_Right] = 0
                            else:
                                buttonList[idTrigger_Right] = 1
                """ Obtenemos el comando """
                if buttonList in ROBOT_LIST_COMMANDS[1]:
                    # Busca el elemento de orden del buttonlist
                    id = ROBOT_LIST_COMMANDS[1].index(buttonList)
                    # Obtenemos el comando
                    command = ROBOT_LIST_COMMANDS[0][id]
                    # Imprimos si es nuevo comando
                    if oldCommand != command:
                        print("Comando {} presionado [{}]".format(
                            ROBOT_LIST_COMMANDS[2][id], command))
                else:
                    command = ""
                oldCommand = command
                # Actualizamos el valor del comando a la variable global
                self.falseParent._VAR_joystickCommand = command
                clock.tick(500)  # Genera delay constante
            except Exception as e:
                print("[ERROR {}]".format(e))
                time.sleep(2)

