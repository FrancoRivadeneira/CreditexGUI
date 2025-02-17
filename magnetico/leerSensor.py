import serial
import time

# Configurar la comunicación con el sensor magnético
magnetico = serial.Serial('COM8', 9600, timeout=1)

# Enviar comandos de inicialización al sensor magnético
magnetico.write(b'TMS 0\r')
magnetico.write(b'# C\r')
magnetico.write(b'?TS\r')
magnetico.write(b'?D\r')

magnetico.write(b'# 10\r')
id=0
while True:
    time.sleep(0.5)
    if magnetico.in_waiting:
        data = magnetico.read_until(b'\r\r').decode().strip()
        print(f"Hola: {data}")
        magnetico.flushInput()
        # time.sleep(1)
        id=id+1
        try:
            cmd, value = data.split('=')
            # print(f"Comando: {cmd}, Valor: {value}")
            pos= value.split(':')
            # if int(pos[0])<1:
            #     print("es a la menos")
            if(int(pos[0])<=-1 and int(pos[0])>=-19 and cmd=='TS'):
                print(f"{id}-{cmd}: Esta al Centro")
                pass
        except ValueError as e:
            print(f"Error de lectura: {e} | Datos recibidos: {data}")
        
