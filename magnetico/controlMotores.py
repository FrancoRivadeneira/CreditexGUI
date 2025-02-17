import serial
import time

# Configurar la comunicación con el sensor magnético
magnetico = serial.Serial('COM8', 9600, timeout=1)

# Enviar comandos de inicialización al sensor magnético
magnetico.write(b'# C\r')
magnetico.write(b'?TS\r')
magnetico.write(b'?D\r')
magnetico.write(b'# 10\r')

while True:
    if magnetico.in_waiting:
        data = magnetico.read_until(b'\r\r').decode().strip()
        magnetico.flushInput()

        try:
            cmd, value = data.split('=')
            if cmd == 'TS':
                valor1, _ = map(int, value.split(':'))  # Extraer el primer valor
                
                # Determinar la dirección
                if(valor1<=-1 and valor1>=-19):
                    direccion = "avanza"
                
                elif valor1 < -19:

                    direccion = "izquierda"
                elif valor1 > -1 and valor1!=0:
                    direccion = "derecha"
                elif valor1 == 0:
                    direccion = "detener"
                
                
                print(f"Valor: {valor1}, Dirección: {direccion}")

        except ValueError as e:
            print(f"Error de lectura: {e} | Datos recibidos: {data}")
