# Version: 1.0
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import pygame
pygame.init()
pygame.joystick.init()


class ControlThread(QThread):
    # Se declara el evento para conectar joysticks
    joystick_moved = pyqtSignal(float, int)
    # Se declara evento para conectar botones
    joystick_pressed = pyqtSignal(int)
    data_received = pyqtSignal(str)
    # Constructor de la clase del control

    def __init__(self):
        super().__init__()
        pygame.init()
        pygame.joystick.init()

    def run(self):
        # llamamos las variables globales que albergan la direccion y la direccion anterior.
        global forward
        global distancia_abs
        global d_prev
        global posicion_label
        global reiniciar_flag
        global primera_vez
        global f_d
        global encoder
        # CAMBIAR COM Y VELOCIDAD DE COMUNICACION
        # puerto_serie = serial.Serial('COM8',9600)
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        # Se cambia la velocidad de trasmisión
        # puerto_serie.baudrate=115200 #Una vez se corrija esto ya se quita
        while self.isRunning:
            ejecutando = True
            vel = 100
            bandera = True

            while ejecutando:
                for evento in pygame.event.get():
                    if evento.type == pygame.QUIT:
                        ejecutando = False

                # Comando del Joystick para retroceder
                if self.controller.get_button(0):
                    dato = '$OAX9X1' + str(vel) + '1' + str(vel)
                    print("atras: " + str(dato))
                    # puerto_serie.write(dato.encode())

                # Comando del Joystick para Avanzar
                elif self.controller.get_button(1):
                    dato = '$OAX9X0' + str(vel) + '0' + str(vel)
                    print("delante: " + str(dato))
                    # puerto_serie.write(dato.encode())

                # Comando del Joystick para Bajar todos los Flippers Juntos
                elif self.controller.get_button(2):

                    dato = '$OAX9F1' + str(vel) + '1' + str(vel)

                    # dato='$OAX9F11001100'
                    print("flipper abajo " + str(dato))
                    # puerto_serie.write(dato.encode())

                # Comando del Joystick para Subir todos los Flippers Juntos
                elif self.controller.get_button(3):

                    dato = '$OAX9F0' + str(vel) + '0' + str(vel)
                    # dato='$OAX9F01000100'
                    print("flipper arriba " + str(dato))
                    # puerto_serie.write(dato.encode())

                # Comando del Joystick para controlar un flipper dependiendo del Sentido
                elif self.controller.get_button(4):
                    dato = '$OAX9F0' + str(vel) + '0000'
                    # dato='$OAX9F01000000'
                    print("Flipper Izquierdo Abajo")
                    # puerto_serie.write(dato.encode())

                # Comando del Joystick para controlar un flipper dependiendo del Sentido
                elif self.controller.get_button(5):
                    dato = '$OAX9F1' + str(vel) + '0000'
                    # dato='$OAX9F11000000'
                    print("Flipper Derecho Arriba")
                    # puerto_serie.write(dato.encode())

                elif self.controller.get_button(6):
                    if bandera == True:
                        bandera = False
                    else:
                        bandera = True
                    print("Cambiar Sentido")
                    # puerto_serie.write(dato.encode())

                elif self.controller.get_button(7):
                    dato = '$OAX2M1'
                    print("Prender Motores")
                    # puerto_serie.write(dato.encode())

                elif self.controller.get_button(8):
                    dato = '$OAX9F0' + '0000' + str(vel)
                    # dato='$OAX9F00000100'
                    print("Flipper Izquierdo Trasero")
                    # puerto_serie.write(dato.encode())

                elif self.controller.get_button(9):
                    dato = '$OAX9F0' + '0001' + str(vel)
                    # dato='$OAX9F00001100'
                    print("Flipper Derecho Trasero")
                    # puerto_serie.write(dato.encode())

                # Asumiendo que el eje 0 representa el joystick izquierdo horizontal
                elif self.controller.get_axis(0) > 0.5:
                    dato = '$OAX9X1' + str(vel) + '0' + str(vel)
                    print("Giro a la derecha")
                    # puerto_serie.write(dato.encode())
                # Asumiendo que el eje 0 representa el joystick izquierdo horizontal
                elif self.controller.get_axis(0) < -0.5:
                    dato = '$OAX9X0' + str(vel) + '1' + str(vel)
                    print("Giro a la Izquierda")
                    # puerto_serie.write(dato.encode())

                # Asumiendo que el eje 0 representa el joystick izquierdo horizontal
                elif self.controller.get_axis(1) > 0.5:

                    if vel > 100:
                        vel = vel-50
                    else:
                        vel = vel
                    Str_dato = "Bajar velocidad: " + str(vel)

                    dato = 'Y'
                    print(Str_dato)
                    # puerto_serie.write(dato.encode())

                # Asumiendo que el eje 0 representa el joystick izquierdo horizontal
                elif self.controller.get_axis(1) < -0.5:

                    if vel < 400:
                        vel = vel+50
                    else:
                        vel = vel
                    Str_dato = "Subir velocidad: " + str(vel)

                    print(Str_dato)
                    # puerto_serie.write(dato.encode())

                pygame.time.delay(140)

    def stop(self):
        self.running = False

