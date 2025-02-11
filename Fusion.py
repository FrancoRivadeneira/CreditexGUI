import cv2
import socket
import imutils
import numpy as np
import time
import base64
import threading
import queue
import os
import serial
import logging
import signal
import pickle
import re
from typing import Union

""" LOGGING FORMAT """
FORMAT = "%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s"
logging.basicConfig(level=logging.INFO, format=FORMAT)

""" INFO CONNECTIONS """
IP_ADDR = "192.168.0.10"
PORT_JOYSTICK = 8672

NUCLEO_PORT = '/dev/ttyUSB0'
NUCLEO_BAUDRATE = 115200


vel=100
Battery=10
## Commands info
comandoList = [
    ["$OAX9X00500050", "Avanzar"],
    ["$OAX9X10501050", "Retroceder"],
    ["$OAX9X10500050", "Giro Horario"],
    ["$OAX9X00501050", "Giro Antihorario"],
    ["$OAX1Y", "Prender Luces"],
    ["$OAX3JR2", "Apagar Luces"],
    ["$OAX3JX1", "Aumentar Velocidad"],
    ["$OAX3JX2", "Disminuir Velocidad"],
    ["$OAX9F00001050", "Flipper Tra Ar"],
    ["$OAX9F00000050", "Flipper Tra Ab"],
    ["$OAX1M", "Activar Motores"],
    ["$OAX9F10501000", "Flipper Del Ar"],
    ["$OAX9F00501000", "Flipper Del Ab"],
    ["$OAX3JBE", "Boton de emergencia"],
    ["$OAX1B","Vateria"]
]

## Convertimos a un diccionario para facilitar la búsqueda
comandos_dict = dict(comandoList)

# Variables compartidas entre hilos
exit_flag = threading.Event()
_FLAG_nucleoConnected = False
_FLAG_SocketConnected = False
_VAR_nucleoSerial = None
_VAR_robotTrama = "NADA"
_FLAG_newData = False

# Configuración de colas para el intercambio de datos entre hilos
q1 = queue.Queue(maxsize=10)
q2 = queue.Queue(maxsize=10)
q3 = queue.Queue(maxsize=10)
q4 = queue.Queue(maxsize=10)

BUFF_SIZE = 65536

# Configuración de sockets y cámaras
def escribir_datos_al_archivo(archivo, datos):
    with open(archivo, 'w') as file:
        file.write(f"{datos}\n")

def sendCommandToMicrocontroller(comando)->Union[str,None]:
    global vel
    global _VAR_nucleoSerial, _FLAG_nucleoConnected
    if comando not in comandos_dict:
        logging.info("Comando \"{}\" no registrado, perdido en los datos...".format(comando))
        return
    comandoRobot = comandos_dict[comando]
    print(comandoRobot)
    try:
        if comando=="$OAX3JX1":
            if vel<400:vel=vel+50
            else:vel=vel
            print(vel)
        elif comando=="$OAX3JX2":
            if vel>100:vel=vel-50
            else:vel=vel
            print(vel)
        
        if comando=="$OAX9X00500050":comando="$OAX9X0"+str(vel)+"0"+str(vel)
        elif comando=="$OAX9X10501050":comando="$OAX9X1"+str(vel)+"1"+str(vel)
        _VAR_nucleoSerial.flush()
        _VAR_nucleoSerial.write(comando.encode())     
        logging.info("Comando {} - \"{}\" enviado al robot!".format(comandoRobot, comando))
    except Exception as e:
        logging.error("Error al enviar el comando {}: {}".format(comandoRobot, e))
        _FLAG_nucleoConnected = False
        
def sendCommandToMicrocontrollerRec(comando)->Union[str,None]:
    global vel
    global _VAR_nucleoSerial, _FLAG_nucleoConnected
    if comando not in comandos_dict:
        logging.info("Comando \"{}\" no registrado, perdido en los datos...".format(comando))
        return
    comandoRobot = comandos_dict[comando]
    print(comandoRobot)
    try:
        if comando=="$OAX3JX1":
            if vel<400:vel=vel+50
            else:vel=vel
            print(vel)
        elif comando=="$OAX3JX2":
            if vel>100:vel=vel-50
            else:vel=vel
            print(vel)
        speed=(100/400)*vel
        print(f"La velocidad es: {speed}")
        message=f"Speed: {speed}"
        escribir_datos_al_archivo("dataint.txt", message)
        if comando=="$OAX9X00500050":comando="$OAX9X0"+str(vel)+"0"+str(vel)
        elif comando=="$OAX9X10501050":comando="$OAX9X1"+str(vel)+"1"+str(vel)
        _VAR_nucleoSerial.flush()
        _VAR_nucleoSerial.write(comando.encode())
        data: str=_VAR_nucleoSerial.read_until().strip().decode('utf-8')
        if not data:
            data=None

        print(data)
        _VAR_nucleoSerial.reset_input_buffer()

        logging.info("Comando {} - \"{}\" enviado al robot!".format(comandoRobot, comando))
    except Exception as e:
        logging.error("Error al enviar el comando {}: {}".format(comandoRobot, e))
        _FLAG_nucleoConnected = False
        data=None

    return data

def SocketConnection():
    global cliente, _FLAG_SocketConnected, _VAR_robotTrama, _FLAG_newData, Battery
    _FLAG_SocketConnected = False

    oldCommand = ""
    s = None  # Variable para almacenar el socket
    while not exit_flag.is_set():
        try:
            if not _FLAG_SocketConnected:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(10)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((IP_ADDR, PORT_JOYSTICK))
                logging.info("Escuchando cliente....")
                s.listen(1)
                cliente, _ = s.accept()
                logging.info("Cliente conectado!")
                _FLAG_SocketConnected = True
            comando=''
            comando = cliente.recv(60)# Eliminar espacios en blanco adicionales alrededor del comando
            comando = pickle.loads(comando).decode()
            
            if comando:
                print("Comando recibido:", comando)

                if comando in comandos_dict:
                    print("El comando está en la lista de comandos permitidos.")
                    if comando=="$OAX1B":
                        Battery=sendCommandToMicrocontrollerRec(comando)
                    else:
                        sendCommandToMicrocontroller(comando)

                    speed=(100/400)*vel
                    print(f"La velocidad es: {speed}")
                    message=f"Speed: {speed}, Battery: {Battery}"

                    escribir_datos_al_archivo("dataint.txt", message)
                else:
                    print("El comando no está en la lista de comandos permitidos.")
                    #logging.info("Comando no encontrado en la lista de comandos permitidos: {}".format(comando))
                oldCommand = comando

            # Enviamos la trama del status del robot por el socket
            cliente.send(pickle.dumps(_VAR_robotTrama))
            time.sleep(0.1)  # Esperamos un breve tiempo para evitar un uso intensivo de CPU
        except Exception as e:
            logging.error("Error en el hilo SocketConnection: {}".format(e))
            _FLAG_SocketConnected = False
            time.sleep(2)
        finally:
            if s:  # Cerramos el socket si está abierto
                s.close()


def readStatusFromMicrocontroller():
    global _VAR_nucleoSerial, _FLAG_nucleoConnected, _VAR_robotTrama, _FLAG_newData
    logging.info("Hilo de nucleo")
    _FLAG_nucleoConnected = False
    while not exit_flag.is_set():
        try:
            if not _FLAG_nucleoConnected:
                _VAR_nucleoSerial = serial.Serial(NUCLEO_PORT, NUCLEO_BAUDRATE, timeout=1)
                _FLAG_nucleoConnected = True
                logging.info("Conectado a la nucleo!")

            #data = _VAR_nucleoSerial.readline().strip()  # Leemos y limpiamos los datos recibidos
            #_VAR_robotTrama = data
            #_FLAG_newData ^= 1
        except Exception as e:
            logging.error("Error en el hilo readStatusFromMicrocontroller: {}".format(e))
            _FLAG_nucleoConnected = False
            _VAR_robotTrama = "NADA"
            time.sleep(1)

def sigint_handler(signum, frame):
    logging.info("Received SIGINT. Exiting gracefully...")
    exit_flag.set()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)

    # Iniciar hilos
    threading.Thread(target=SocketConnection).start()
    threading.Thread(target=readStatusFromMicrocontroller).start()
