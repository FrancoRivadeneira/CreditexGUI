/****************************************************************************************/
/****************************************************************************************/
/*****
PONTIFICIA UNIVERSIDAD CATÓLICA DEL PERÚ
*****/
/*****
FACULTAD DE CIENCIAS E INGENIERÍA
*****/
/*****
PROYECTO DE DISEÑO MECATRÓNICO
*****/
/****************************************************************************************/
/*****
Proyecto: DISEÑO E IMPLEMENTACIÓN DE UN SISTEMA MECATRÓNICO LOGÍSTICO *****/
/*****
PARA EL TRANSPORTE DE CARGAS EN ALMACENES
*****/
/****************************************************************************************/
/*****
Version:
8.0
*****/
/****************************************************************************************/
/****************************************************************************************/
// Librerías
#include "RoboClaw.h"
#include <SoftwareSerial.h>
#include <Servo.h>
// Definición de la dirreción del RoboClaw
#define address 0x80
// Definición de pines
#define stepPin_insercion 6
#define dirPin_insercion 5
#define stepPin_elevacion 4
#define dirPin_elevacion 3
#define endOfLine1 28
#define endOfLine2 34
#define endOfLine3 40
#define endOfLine4 46

// Pin del sistema de inserción/sujeción
// Pin del sistema de inserción/sujeción
// Pin del sistema de elevación
// Pin del sistema de elevación
// Final de carrera 2 (Descenso de la carga)
// Final de carrera 1 (Elevación de la carga)
// Final de carrera 2, Retiro de la carga
// Final de carrera 1, inserción de la carga

// Definición de parámetros del controlador PI del RoboClaw
#define Kp_rc 4.63529
// Coeficientes PID de velocidad
#define Ki_rc 0.77252
// Coeficientes PID de velocidad
#define Kd_rc 0
// Coeficientes PID de velocidad
#define qpps 3600
// Coeficientes PID de velocidad
// Definición de parámetros del controlador P del seguidor de línea
#define Kp 1
// Parámetros PID del seguidor de línea
#define Ki 0
// Parámetros PID del seguidor de línea
// Definición del Arduino SoftwareSerial y RoboClaw
SoftwareSerial serial(10, 11);
// Serial del RoboClaw
RoboClaw roboclaw(&serial, 10000);
// Definición de banderas
bool estado_inicial;
bool estado_BT = true;
bool movimiento;

// Indicador del estado inicial del sistema
// Indicador del estado inicial del BT
// Avance: movimiento = true. Reversa: movimiento = false

// Definición de variables
char dato;
char anaquel;
int speed_E = 1000;
int speed_IS = 500;
int duty;

// Dato recibido por la señal Bluetooth
// Nivel de anaquel
// Velocidad del sistema de elevación
// Velocidad del sistema de sujeción
// El valor del duty tiene signo y el rango es de -32767 a +32767

int porcentaje = 10;
int sensor_mag = 0;
char buffer[10];
bool inicio_E = false;
bool stop_E = false;
bool inicio_IS = false;
bool stop_IS = false;

// Porcentaje de velocidad, paso del 10%.
// Variable que indica el valor leído por el sensor y lo muestra en
// Cadena de mensaje del porcentaje de velocidad y del sensor
// Indica cuando ya se presionó el fin de carrera del sistema de
// Indica cuando ya se presionó el fin de carrera del sistema de
// Indica cuando ya se presionó el fin de carrera del sistema de
// Indica cuando ya se presionó el fin de carrera del sistema de

// Definición de variables de los servomotores
int posicion;
// Variable para controlar la posición de los servomotores
Servo servoMotorIzq;
// Declaramos el objeto del servo izquierdo (el que está al lado del Nema 17)
Servo servoMotorDer;
// Declaramos la variable para controlar el servo derecho
// Definición de variables del sensor magnético
int analogInPin = A0;
// Brinda una señal analógica que indica la posición de la cintamagnética respecto al sensor
int pathPin = 48;
// Indica si se ha detectado la cinta magnética
int leftMarker = 42;
// indica si se ha detectado un marcador que se encuentra a la izquierda de la cinta magnética
int rightMarker = 36;
// indica si se ha detectado un marcador que se encuentra a la derecha de la cinta magnética
int sensorValue;
// Valor digital obtenido del ADC del microcontrolador, está en el rango de 0 a 1024
double sensorValue_voltage;
// Variable utilizada para obtener el valor de voltaje leído por el sensor
bool endOfRoad = true;
// Indica cuando debe de dejar de seguir la línea magnética
// Definición de variables para el control de movimiento del carrito
uint8_t wr, wl;
const float radius = 0.125;
// 125 mm de radio de la rueda
const float Rw = 0.40;
// Separación de las ruedas (40 cm aprox.)
// Algoritmo PID para el seguimiento de la cinta magnética.
double uk, uk_1 = 0, ek, ek_1 = 0, T = 0.25; // uk: Velocidad angular del sistema en rad/s.
double dist, dist_ref = 0;
// Distancia de referencia en voltaje: 0V - 3V (1.5V es el centro de la cinta magnética)
// Variables para realizar una determinada tarea luego de un intervalo X en ms, sin detener la ejecución del resto del algoritmo.
unsigned long previousMillis = 0; // Almacenará la última vez fue ejecutada.
unsigned long currentMillis;
const long interval = 100;
// Intervalo de tiempo para que se realice la tarea
(milisegundos) void setup()
{
    // Inicialización del módulo Bluetooth, RoboClaw y pines de los servomotores
    Serial.begin(9600);
    Serial1.begin(9600);
    roboclaw.begin(38400);

    servoMotorDer.attach(13);
    servoMotorIzq.attach(9);
    // Inicialización de los pines del sensor magnético
    pinMode(pathPin, INPUT);
    pinMode(leftMarker, INPUT);
    pinMode(rightMarker, INPUT);
    // Inicialización de los pines de STEP y DIR de los motores paso a paso del sistema de elevación y del sistema de inserción / sujeción
    pinMode(stepPin_elevacion, OUTPUT);
    pinMode(dirPin_elevacion, OUTPUT);
    pinMode(stepPin_insercion, OUTPUT);
    pinMode(dirPin_insercion, OUTPUT);
    // Inicialización de los pines de los fines de carrera del sistema de elevación y del sistema de inserción / sujeción
    pinMode(endOfLine1, INPUT_PULLUP);
    pinMode(endOfLine2, INPUT_PULLUP);
    pinMode(endOfLine3, INPUT_PULLUP);
    pinMode(endOfLine4, INPUT_PULLUP);
    // Inicialización la posición de los servomotores
    servoMotorIzq.write(0);
    servoMotorDer.write(105);
    // Inicialización los coeficientes PID del controlador del RoboClaw
    // Establece los parámetros PID y máxima velocidad en QPPS: Quadrature pulses per second)
    roboclaw.SetM1VelocityPID(address, Kd_rc, Kp_rc, Ki_rc, qpps);
    roboclaw.SetM2VelocityPID(address, Kd_rc, Kp_rc, Ki_rc, qpps);
}
void loop()
{
    // Verificación de la señal Bluetooth inicial
    if (estado_BT == true)
    {
        if (Serial1.available())
        {
            dato = Serial1.read();
            Serial.write(dato);
            estado_BT = false;
        }
    }
    if (dato == 'A')
    {

        // MODO AUTOMÁTICO

        Serial.println("Modo automático");
        endOfRoad = false;
        while ((anaquel != '1') && (anaquel != '2'))
        {
            while (Serial1.available() == 0)
            {
            }
            anaquel = Serial1.read();
            Serial.write(anaquel);
        }

        do
        {
            if (Serial1.available())
            {

                dato = Serial1.read();
                Serial.write(dato);
            }
            switch (dato)
            {
            case 'I':
                for (posicion = 105; posicion >= 0; posicion -= 1)
                {

                    // Cerrar servos

                    servoMotorIzq.write(105 - posicion);
                    servoMotorDer.write(posicion);
                    delay(20);
                }

                while (!endOfRoad)
                {
                    currentMillis = millis();
                    if (currentMillis - previousMillis >= interval)
                    { // Se lee el dato del sensor magnético
                        cada 100 milisegundos
                            previousMillis = currentMillis;
                        // Verificar si se detectó solo un marcador derecho, de ser el caso, se termina el recorrido automático.
                        if (((digitalRead(rightMarker) == true) && (digitalRead(leftMarker) == false)) ||
                                                 (digitalRead(pathPin) == false))
                        {
                            endOfRoad = true;
                        }
                        // Detección de parada de emergencia
                        if (Serial1.available())
                        {
                            dato = Serial1.read();
                            Serial.write(dato);
                        }
                        if ((dato == 'P') or (dato == 'H'))
                        {
                            break;
                        }
                        // Lectura del valor analógico brindado por el sensor
                        sensorValue = analogRead(analogInPin);
                        // Conversión del valor leído a un rango de voltaje de 0 a 5V.
                        sensorValue_voltage = 5 * (float(sensorValue) / 1023);
                        if (sensorValue_voltage > 1.88)
                        {
                            sensorValue_voltage = 1.88;
                        }
                        else if (sensorValue_voltage < 0.88)
                        {
                            sensorValue_voltage = 0.88;
                        }

                        // Lectura del sensor en mm (estará en un rango de -80 a 80 mm)
                        dist = ((160 / 1) * (sensorValue_voltage - 0.9)) - 80;
                        // Determinación de la acción de control
                        ek = dist_ref - dist;
                        // Algoritmo P:
                        uk = (0.1) * ek;
                        Serial.print("uk: ");
                        Serial.println(uk);
                        Serial.print("M1 (0 - 127): ");
                        Serial.println(wr);
                        Serial.print("M2 (0 - 127): ");
                        Serial.println(wl);
                        Serial.println("El sensor está ACTIVADO");

                        // Para saber si está ingresando al while
                        Serial.print("Lectura del sensor: ");
                        Serial.println(dist);
                        Serial.println(" ");
                        if (digitalRead(rightMarker) == true)
                        {
                            Serial.print("Right marker DETECTADO");
                        }
                        else
                        {
                            Serial.print("Right marker NO DETECTADO");
                        }
                        Serial.println(" ");
                    }

                    // Mandar uk para accionar los motores del RoboClaw.
                    wr = 80 + (Rw / radius) * uk;
                    wl = 80 - (Rw / radius) * uk;
                    // wr_PPS = (M_PI/1024)*wr;
                    // wl_PPS = (M_PI/1024)*wl;
                    // Limitar de la velocidad máxima en PPS
                    if (wr >= 105)
                    {
                        wr = 105;
                    }
                    if (wr <= 55)
                    {
                        wr = 55;
                    }
                    if (wl >= 105)
                    {
                        wl = 105;
                    }
                    if (wl <= 55)
                    {
                        wl = 55;
                    }
                    roboclaw.ForwardBackwardM1(address, wr);
                    roboclaw.ForwardBackwardM2(address, wl);

                    // wr estará entre 0 y 127
                    // wl estará entre 0 y 127

                    // Tranmisión del dato del sensor magnético a la aplicación

                    sensor_mag = dist;
                    sprintf(buffer, " ,%d", sensor_mag);
                    Serial1.write(buffer);
                }
                if ((dato == 'P') or (dato == 'H'))
                {
                    break;
                }
                Serial.println("El sensor está DESACTIVADO");

                // Para saber si está ingresando al while
                roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);
                delay(500);
                endOfRoad = false;

                // Esta variable indica que el vehículo hará un segundo recorrido

                if (anaquel == '1')
                {
                    // Aquí inicia el proceso de inserción de la carga en el anaquel
                    inicio_E = false;
                    if (inicio_E != true)
                    { // Se verifica si el sistema de elevación no llegó al límite inferior
                        digitalWrite(dirPin_elevacion, LOW);
                        while (inicio_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine1) == false)
                            {
                                inicio_E = true;
                            }
                        }
                    }
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    // Una vez se haya colocado la carga en el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    // Aquí se termina el proceso de inserción de la carga en el anaquel
                }
                else if (anaquel == '2')
                {
                    // Aquí inicia el proceso de inserción de la carga en el anaquel
                    stop_E = false;
                    if (stop_E != true)
                    {

                        // Se verifica si el sistema de elevación no llegó al límite

                        superior
                            digitalWrite(dirPin_elevacion, HIGH);
                        while (stop_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine2) == false)
                            {
                                stop_E = true;
                            }
                        }
                    }
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    // Una vez se haya colocado la carga en el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    inicio_E = false;
                    if (inicio_E != true)
                    { // Si el sistema de elevación se encuentra en una posición
                        superior, pasa a descender
                                      digitalWrite(dirPin_elevacion, LOW);

                        while (inicio_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine1) == false)
                            {
                                inicio_E = true;
                            }
                        }
                    }
                    // Aquí se termina el proceso de inserción de la carga en el anaquel
                }
                for (posicion = 105; posicion >= 0; posicion -= 1)
                {

                    // Cerrar servos

                    servoMotorIzq.write(105 - posicion);
                    servoMotorDer.write(posicion);
                    delay(20);
                }
                while (!endOfRoad)
                {
                    currentMillis = millis();
                    if (currentMillis - previousMillis >= interval)
                    { // Se lee el dato del sensor magnético
                        cada 500 milisegundos
                            previousMillis = currentMillis;
                        // Verificamos si se detectó dos marcadores en la cinta magnética, de ser el caso, se
                        termina el recorrido automático.if (((digitalRead(rightMarker) == true && digitalRead(leftMarker) == true))(digitalRead(pathPin) == false))
                        {
                            endOfRoad = true;
                        }
                        // Detección de parada de emergencia
                        if (Serial1.available())
                        {
                            dato = Serial1.read();
                            Serial.write(dato);
                        }
                        if ((dato == 'P') or (dato == 'H'))
                        {
                            break;
                        }
                        // Lectura del valor analógico brindado por el sensor
                        sensorValue = analogRead(analogInPin);
                        // Conversión del valor leído a un rango de voltaje de 0 a 5V.

                        ||

                            sensorValue_voltage = 5 * (float(sensorValue) / 1023);
                        if (sensorValue_voltage > 1.88)
                        {
                            sensorValue_voltage = 1.88;
                        }
                        else if (sensorValue_voltage < 0.88)
                        {
                            sensorValue_voltage = 0.88;
                        }
                        // Lectura del sensor en mm (estará en un rango de -80 a 80 mm)
                        dist = ((160 / 1) * (sensorValue_voltage - 0.9)) - 80;
                        // Determinamos la acción de control
                        ek = dist_ref - dist;
                        // Algoritmo P:
                        uk = (0.1) * ek;

                        // uk = Kp*ek = (14/80)*ek = (0.175)*ek?? --> -14 <= uk <= 14

                        Serial.print("uk: ");
                        Serial.println(uk);
                        Serial.print("M1 (0 - 127): ");
                        Serial.println(wr);
                        Serial.print("M2 (0 - 127): ");
                        Serial.println(wl);
                        Serial.println("El sensor está ACTIVADO");

                        // Para saber si está ingresando al

                        while
                            Serial.print("Lectura del sensor: ");
                        Serial.println(dist);
                        Serial.println(" ");
                        if (digitalRead(rightMarker) == true)
                        {
                            Serial.print("Right marker DETECTADO");
                        }
                        else
                        {
                            Serial.print("Right marker NO DETECTADO");
                        }
                        Serial.println(" ");
                    }
                    // Mandar uk para accionar los motores del RoboClaw
                    wr = 80 + (Rw / radius) * uk;
                    wl = 80 - (Rw / radius) * uk;
                    // Limitar la velocidad máxima en PPS??
                    if (wr >= 105)
                    {
                        wr = 105;
                    }
                    if (wr <= 55)
                    {
                        wr = 55;
                    }
                    if (wl >= 105)
                    {
                        wl = 105;
                    }
                    if (wl <= 55)
                    {
                        wl = 55;
                    }

                    // Antes: v/radius, Ahora: 64 + 16 = 80

                    roboclaw.ForwardBackwardM1(address, wr);
                    roboclaw.ForwardBackwardM2(address, wl);

                    // wr estará entre 0 y 127
                    // wl estará entre 0 y 127

                    // Tranmisión del dato del sensor magnético a la aplicación
                    sensor_mag = dist;
                    sprintf(buffer, " ,%d", sensor_mag);
                    Serial1.write(buffer);
                }
                if ((dato == 'P') or (dato == 'H'))
                {
                    break;
                }
                Serial.println("El sensor está DESACTIVADO");

                // Para saber si está ingresando al

                while
                    roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);
                delay(500);
                dato = '#';
                anaquel = '0';
                break;
            case 'R':
                for (posicion = 105; posicion >= 0; posicion -= 1)
                {

                    // Cerrar servos

                    servoMotorIzq.write(105 - posicion);
                    servoMotorDer.write(posicion);
                    delay(20);
                }
                while (!endOfRoad)
                {
                    currentMillis = millis();
                    if (currentMillis - previousMillis >= interval)
                    { // Se lee el dato del sensor magnético
                        cada 500 milisegundos
                            previousMillis = currentMillis;
                        // Verificar si se detectó un marcador derecho en la cinta magnética, de ser el caso,
                        termina el recorrido automático.if (((digitalRead(rightMarker) == true && digitalRead(leftMarker) == false)) ||
                                                            (digitalRead(pathPin) == false))
                        {
                            endOfRoad = true;
                        }
                        // Detección de parada de emergencia
                        if (Serial1.available())
                        {

                            dato = Serial1.read();
                            Serial.write(dato);
                        }
                        if ((dato == 'P') or (dato == 'H'))
                        {
                            break;
                        }
                        // Lectura del valor analógico brindado por el sensor
                        sensorValue = analogRead(analogInPin);
                        // Conversión el valor leido a un rango de voltaje de 0 a 5V.
                        sensorValue_voltage = 5 * (float(sensorValue) / 1023);
                        if (sensorValue_voltage > 1.88)
                        {
                            sensorValue_voltage = 1.88;
                        }
                        else if (sensorValue_voltage < 0.88)
                        {
                            sensorValue_voltage = 0.88;
                        }
                        // Lectura del sensor en mm (estará en un rango de -80 a 80 mm)
                        dist = ((160 / 1) * (sensorValue_voltage - 0.9)) - 80;
                        // Determinar la acción de control
                        ek = dist_ref - dist;
                        // Algoritmo P:
                        uk = (0.1) * ek;
                        Serial.print("uk: ");
                        Serial.println(uk);
                        Serial.print("M1 (0 - 127): ");
                        Serial.println(wr);
                        Serial.print("M2 (0 - 127): ");
                        Serial.println(wl);
                        Serial.println("El sensor está ACTIVADO");

                        // Para saber si está ingresando al

                        while
                            Serial.print("Lectura del sensor: ");
                        Serial.println(dist);
                        Serial.println(" ");
                        if (digitalRead(rightMarker) == true)
                        {
                            Serial.print("Right marker DETECTADO");
                        }
                        else
                        {
                            Serial.print("Right marker NO DETECTADO");
                        }
                        Serial.println(" ");
                    }
                    // Mandar uk para accionar los motores del RoboClaw.
                    wr = 80 + (Rw / radius) * uk;
                    // Antes: v/radius, Ahora: 64 + 16 = 80
                    wl = 80 - (Rw / radius) * uk;

                    // Limitar la velocidad máxima en PPS??
                    if (wr >= 105)
                    {
                        wr = 105;
                    }
                    if (wr <= 55)
                    {
                        wr = 55;
                    }
                    if (wl >= 105)
                    {
                        wl = 105;
                    }
                    if (wl <= 55)
                    {
                        wl = 55;
                    }
                    roboclaw.ForwardBackwardM1(address, wr);
                    roboclaw.ForwardBackwardM2(address, wl);

                    // wr estará entre 0 y 127
                    // wl estará entre 0 y 127

                    // Tranmisión del dato del sensor magnético a la aplicación
                    sensor_mag = dist;
                    sprintf(buffer, " ,%d", sensor_mag);
                    Serial1.write(buffer);
                }
                if ((dato == 'P') or (dato == 'H'))
                {
                    break;
                }
                Serial.println("El sensor está DESACTIVADO");

                // Para saber si está ingresando al

                while
                    roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);
                delay(500);
                endOfRoad = false;
                if (anaquel == '1')
                {
                    // Aquí inicia el proceso de retiro de la carga desde el anaquel
                    inicio_E = false;
                    if (inicio_E != true)
                    { // Se verifica si el sistema de elevación no llegó al límite
                        inferior
                            digitalWrite(dirPin_elevacion, LOW);
                        while (inicio_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine1) == false)
                            {
                                inicio_E = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 105; posicion >= 0; posicion -= 1)
                    {

                        // Cerrar servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    // Una vez se haya retirado la carga desde el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    // Aquí se termina el retiro de la carga desde el anaquel
                }
                else if (anaquel == '2')
                {
                    // Aquí se inicia el retiro de la carga desde el anaquel
                    stop_E = false;
                    if (stop_E != true)
                    {

                        // Se verifica si el sistema de elevación no llegó al límite

                        superior
                            digitalWrite(dirPin_elevacion, HIGH);
                        while (stop_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine2) == false)
                            {
                                stop_E = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 105; posicion >= 0; posicion -= 1)
                    {
                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);

                        // Cerrar servos

                        delay(20);
                    }
                    // Una vez se haya retirado la carga desde el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    inicio_E = false;
                    if (inicio_E != true)
                    { // Si el sistema de elevación se encuentra en una posición
                        superior, pasa a descender
                                      digitalWrite(dirPin_elevacion, LOW);
                        while (inicio_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine1) == false)
                            {
                                inicio_E = true;
                            }
                        }
                    }
                    // Aquí se termina el retiro de la carga desde el anaquel
                }
                while (!endOfRoad)
                {
                    currentMillis = millis();
                    if (currentMillis - previousMillis >= interval)
                    { // Se lee el dato del sensor magnético
                        cada 500 milisegundos
                            previousMillis = currentMillis;

                        // Verificar si se detectó dos marcadores en la cinta magnética, de ser el caso, se
                        termina el recorrido automático.if (((digitalRead(rightMarker) == true) && (digitalRead(leftMarker) == true)) ||
                                                            (digitalRead(pathPin) == false))
                        {
                            endOfRoad = true;
                        }
                        // Detección de parada de emergencia
                        if (Serial1.available())
                        {
                            dato = Serial1.read();
                            Serial.write(dato);
                        }
                        if ((dato == 'P') or (dato == 'H'))
                        { // PARADA DE EMERGENCIA
                            break;
                        }
                        // Lectura del valor analógico brindado por el sensor
                        sensorValue = analogRead(analogInPin);
                        // Conversión del valor leido a un rango de voltaje de 0 a 5V.
                        sensorValue_voltage = 5 * (float(sensorValue) / 1023);
                        if (sensorValue_voltage > 1.88)
                        {
                            sensorValue_voltage = 1.88;
                        }
                        else if (sensorValue_voltage < 0.88)
                        {
                            sensorValue_voltage = 0.88;
                        }
                        // Lectura del sensor en mm (estará en un rango de -80 a 80 mm)
                        dist = ((160 / 1) * (sensorValue_voltage - 0.9)) - 80;
                        // Determinamos la acción de control
                        ek = dist_ref - dist;
                        // Algoritmo P:
                        uk = (0.1) * ek;
                        Serial.print("uk: ");
                        Serial.println(uk);
                        Serial.print("M1 (0 - 127): ");
                        Serial.println(wr);
                        Serial.print("M2 (0 - 127): ");
                        Serial.println(wl);
                        Serial.println("El sensor está ACTIVADO");
                        while
                            Serial.print("Lectura del sensor: ");
                        Serial.println(dist);
                        Serial.println(" ");
                        if (digitalRead(rightMarker) == true)
                        {
                            Serial.print("Right marker DETECTADO");

                            // Para saber si está ingresando al
                        }
                        else
                        {
                            Serial.print("Right marker NO DETECTADO");
                        }
                        Serial.println(" ");
                    }
                    wr = 80 + (Rw / radius) * uk;
                    wl = 80 - (Rw / radius) * uk;

                    // Antes: v/radius, Ahora: 64 + 16 = 80

                    // Limitar la velocidad máxima en PPS??
                    if (wr >= 105)
                    {
                        wr = 105;
                    }
                    if (wr <= 55)
                    {
                        wr = 55;
                    }
                    if (wl >= 105)
                    {
                        wl = 105;
                    }
                    if (wl <= 55)
                    {
                        wl = 55;
                    }
                    roboclaw.ForwardBackwardM1(address, wr);
                    roboclaw.ForwardBackwardM2(address, wl);

                    // wr estará entre 0 y 127
                    // wl estará entre 0 y 127

                    // Tranmisión del dato del sensor magnético a la aplicación
                    sensor_mag = dist;
                    sprintf(buffer, " ,%d", sensor_mag);
                    Serial1.write(buffer);
                }
                if ((dato == 'P') or (dato == 'H'))
                {

                    // PARADA DE EMERGENCIA

                    break;
                }
                Serial.println("El sensor está DESACTIVADO");

                // Para saber si está ingresando al

                while
                    roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);
                delay(500);
                dato = '#';
                anaquel = '0';
                break;
            case 'C':
                for (posicion = 105; posicion >= 0; posicion -= 1)
                {
                    servoMotorIzq.write(105 - posicion);

                    // Cerrar servos

                    servoMotorDer.write(posicion);
                    delay(20);
                }
                while (!endOfRoad)
                {
                    currentMillis = millis();
                    if (currentMillis - previousMillis >= interval)
                    { // Se lee el dato del sensor magnético
                        cada 500 milisegundos
                            previousMillis = currentMillis;
                        // Verificar si se detectó un marcador derecho en la cinta magnética, de ser el caso,
                        termina el recorrido automático.if (((digitalRead(rightMarker) == true && digitalRead(leftMarker) == false)) ||
                                                            (digitalRead(pathPin) == false))
                        {
                            endOfRoad = true;
                        }
                        // Detección de parada de emergencia
                        if (Serial1.available())
                        {
                            dato = Serial1.read();
                            Serial.write(dato);
                        }
                        if ((dato == 'P') or (dato == 'H'))
                        {
                            break;
                        }
                        // Lectura del valor analógico brindado por el sensor
                        sensorValue = analogRead(analogInPin);
                        // Conversión el valor leido a un rango de voltaje de 0 a 5V.
                        sensorValue_voltage = 5 * (float(sensorValue) / 1023);
                        if (sensorValue_voltage > 1.88)
                        {
                            sensorValue_voltage = 1.88;
                        }
                        else if (sensorValue_voltage < 0.88)
                        {
                            sensorValue_voltage = 0.88;
                        }
                        // Lectura del sensor en mm (estará en un rango de -80 a 80 mm)
                        dist = ((160 / 1) * (sensorValue_voltage - 0.9)) - 80;
                        // Determinar la acción de control
                        ek = dist_ref - dist;
                        // Algoritmo P:
                        uk = (0.1) * ek;
                        Serial.print("uk: ");

                        Serial.println(uk);
                        Serial.print("M1 (0 - 127): ");
                        Serial.println(wr);
                        Serial.print("M2 (0 - 127): ");
                        Serial.println(wl);
                        Serial.println("El sensor está ACTIVADO");

                        // Para saber si está ingresando al

                        while
                            Serial.print("Lectura del sensor: ");
                        Serial.println(dist);
                        Serial.println(" ");
                        if (digitalRead(rightMarker) == true)
                        {
                            Serial.print("Right marker DETECTADO");
                        }
                        else
                        {
                            Serial.print("Right marker NO DETECTADO");
                        }
                        Serial.println(" ");
                    }
                    // Mandar uk para accionar los motores del RoboClaw.
                    wr = 80 + (Rw / radius) * uk;
                    // Antes: v/radius, Ahora: 64 + 16 = 80
                    wl = 80 - (Rw / radius) * uk;
                    // Limitar la velocidad máxima en PPS??
                    if (wr >= 105)
                    {
                        wr = 105;
                    }
                    if (wr <= 55)
                    {
                        wr = 55;
                    }
                    if (wl >= 105)
                    {
                        wl = 105;
                    }
                    if (wl <= 55)
                    {
                        wl = 55;
                    }
                    roboclaw.ForwardBackwardM1(address, wr);
                    roboclaw.ForwardBackwardM2(address, wl);

                    // wr estará entre 0 y 127
                    // wl estará entre 0 y 127

                    // Tranmisión del dato del sensor magnético a la aplicación
                    sensor_mag = dist;
                    sprintf(buffer, " ,%d", sensor_mag);
                    Serial1.write(buffer);
                }
                if ((dato == 'P') or (dato == 'H'))
                {
                    break;
                }
                Serial.println("El sensor está DESACTIVADO");
                while

                    // Para saber si está ingresando al

                    roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);
                delay(500);
                endOfRoad = false;
                if (anaquel == '1')
                {

                    // Transporta la caja del nivel 1 al nivel 2 del anaquel

                    // Aquí se inicia el retiro de la caja del nivel 1 del anaquel
                    inicio_E = false;
                    if (inicio_E != true)
                    { // Se verifica si el sistema de elevación no llegó al límite
                        inferior
                            digitalWrite(dirPin_elevacion, LOW);
                        while (inicio_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine1) == false)
                            {
                                inicio_E = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 105; posicion >= 0; posicion -= 1)
                    {
                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);

                        // Cerrar servos

                        delay(20);
                    }
                    // Una vez se haya retirado la carga desde el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    delay(2000);
                    // Aquí inicia el proceso de inserción de la carga en el nivel 2 del anaquel
                    stop_E = false;
                    if (stop_E != true)
                    {

                        // Se verifica si el sistema de elevación no llegó al límite

                        superior
                            digitalWrite(dirPin_elevacion, HIGH);
                        while (stop_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine2) == false)
                            {
                                stop_E = true;
                            }
                        }
                    }
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {

                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    // Una vez se haya colocado la carga en el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    inicio_E = false;
                    if (inicio_E != true)
                    { // Si el sistema de elevación se encuentra en una posición
                        superior, pasa a descender
                                      digitalWrite(dirPin_elevacion, LOW);
                        while (inicio_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine1) == false)
                            {
                                inicio_E = true;
                            }
                        }
                    }
                    for (posicion = 105; posicion >= 0; posicion -= 1)
                    {

                        // Cerrar servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    // Aquí se termina el proceso de inserción de la carga en el anaquel
                }
                else if (anaquel == '2')
                {

                    // Transporta la caja del nivel 2 al nivel 1 del anaquel

                    // Aquí se inicia el retiro de la caja del nivel 2 desde el anaquel
                    stop_E = false;
                    if (stop_E != true)
                    {

                        // Se verifica si el sistema de elevación no llegó al límite

                        superior
                            digitalWrite(dirPin_elevacion, HIGH);
                        while (stop_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine2) == false)
                            {
                                stop_E = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 105; posicion >= 0; posicion -= 1)
                    {

                        // Cerrar servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    // Una vez se haya retirado la carga desde el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    inicio_E = false;
                    if (inicio_E != true)
                    { // Si el sistema de elevación se encuentra en una posición
                        superior, pasa a descender
                                      digitalWrite(dirPin_elevacion, LOW);
                        while (inicio_E != true)
                        {
                            digitalWrite(stepPin_elevacion, HIGH);
                            delayMicroseconds(speed_E);
                            digitalWrite(stepPin_elevacion, LOW);
                            delayMicroseconds(speed_E);
                            if (digitalRead(endOfLine1) == false)
                            {
                                inicio_E = true;
                            }
                        }
                    }
                    // Aquí se termina el retiro de la caja desde el anaquel
                    delay(2000);

                    // Aquí se inicia la inserción de la caja en el nivel 1 del anaquel
                    inicio_IS = false;
                    if (inicio_IS != true)
                    {

                        // Se verifica si el sistema de inserción no se extendió

                        digitalWrite(dirPin_insercion, HIGH);
                        while (inicio_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine3) == false)
                            {
                                inicio_IS = true;
                            }
                        }
                    }
                    for (posicion = 0; posicion <= 105; posicion += 1)
                    {

                        // Abrir servos

                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }
                    // Una vez se haya colocado la carga en el anaquel, se espera 2 segundos
                    // Para que los subsistemas regresen a su posición inicial
                    delay(2000);
                    stop_IS = false;
                    if (stop_IS != true)
                    { // Si el sistema de inserción se encuentra extendido, pasa a
                        retraerse
                            digitalWrite(dirPin_insercion, LOW);
                        while (stop_IS != true)
                        {
                            digitalWrite(stepPin_insercion, HIGH);
                            delayMicroseconds(speed_IS);
                            digitalWrite(stepPin_insercion, LOW);
                            delayMicroseconds(speed_IS);
                            if (digitalRead(endOfLine4) == false)
                            {
                                stop_IS = true;
                            }
                        }
                    }
                    for (posicion = 105; posicion >= 0; posicion -= 1)
                    {
                        servoMotorIzq.write(105 - posicion);
                        servoMotorDer.write(posicion);
                        delay(20);
                    }

                    // Cerrar servos

                    // Aquí se termina el proceso de inserción de la carga en el anaquel
                }
                while (!endOfRoad)
                {
                    currentMillis = millis();
                    if (currentMillis - previousMillis >= interval)
                    { // Se lee el dato del sensor magnético
                        cada 500 milisegundos
                            previousMillis = currentMillis;
                        // Verificar si se detectó dos marcadores en la cinta magnética, de ser el caso, se
                        termina el recorrido automático.if (((digitalRead(rightMarker) == true) && (digitalRead(leftMarker) == true)) ||
                                                            (digitalRead(pathPin) == false))
                        {
                            endOfRoad = true;
                        }
                        // Detección de parada de emergencia
                        if (Serial1.available())
                        {
                            dato = Serial1.read();
                            Serial.write(dato);
                        }
                        if ((dato == 'P') or (dato == 'H'))
                        { // PARADA DE EMERGENCIA
                            break;
                        }
                        // Lectura del valor analógico brindado por el sensor
                        sensorValue = analogRead(analogInPin);
                        // Conversión del valor leido a un rango de voltaje de 0 a 5V.
                        sensorValue_voltage = 5 * (float(sensorValue) / 1023);
                        if (sensorValue_voltage > 1.88)
                        {
                            sensorValue_voltage = 1.88;
                        }
                        else if (sensorValue_voltage < 0.88)
                        {
                            sensorValue_voltage = 0.88;
                        }
                        // Lectura del sensor en mm (estará en un rango de -80 a 80 mm)
                        dist = ((160 / 1) * (sensorValue_voltage - 0.9)) - 80;
                        // Determinamos la acción de control
                        ek = dist_ref - dist;
                        // Algoritmo P:
                        uk = (0.1) * ek;
                        Serial.print("uk: ");

                        Serial.println(uk);
                        Serial.print("M1 (0 - 127): ");
                        Serial.println(wr);
                        Serial.print("M2 (0 - 127): ");
                        Serial.println(wl);
                        Serial.println("El sensor está ACTIVADO");

                        // Para saber si está ingresando al

                        while
                            Serial.print("Lectura del sensor: ");
                        Serial.println(dist);
                        Serial.println(" ");
                        if (digitalRead(rightMarker) == true)
                        {
                            Serial.print("Right marker DETECTADO");
                        }
                        else
                        {
                            Serial.print("Right marker NO DETECTADO");
                        }
                        Serial.println(" ");
                    }
                    wr = 80 + (Rw / radius) * uk;
                    wl = 80 - (Rw / radius) * uk;

                    // Antes: v/radius, Ahora: 64 + 16 = 80

                    // Limitar la velocidad máxima en PPS??
                    if (wr >= 105)
                    {
                        wr = 105;
                    }
                    if (wr <= 55)
                    {
                        wr = 55;
                    }
                    if (wl >= 105)
                    {
                        wl = 105;
                    }
                    if (wl <= 55)
                    {
                        wl = 55;
                    }
                    roboclaw.ForwardBackwardM1(address, wr);
                    roboclaw.ForwardBackwardM2(address, wl);

                    // wr estará entre 0 y 127
                    // wl estará entre 0 y 127

                    // Tranmisión del dato del sensor magnético a la aplicación
                    sensor_mag = dist;
                    sprintf(buffer, " ,%d", sensor_mag);
                    Serial1.write(buffer);
                }
                if ((dato == 'P') or (dato == 'H'))
                {

                    // PARADA DE EMERGENCIA

                    break;
                }
                Serial.println("El sensor está DESACTIVADO");
                while
                    roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);

                // Para saber si está ingresando al

                delay(500);
                dato = '#';
                anaquel = '0';
                break;
            case 'H':
                dato = 'H';
                roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);
                anaquel = '0';
                break;
            case 'P':
            case '#':
            default:
                // Modo espera
                roboclaw.ForwardBackwardM1(address, 64);
                roboclaw.ForwardBackwardM2(address, 64);
                anaquel = '0';
                break;
            }
        }

        while ((dato != 'M') && (dato != 'H')); // Se ejecuta el modo automático hasta que se indique la operación en modo manual
        roboclaw.ForwardBackwardM1(address, 64);
        roboclaw.ForwardBackwardM2(address, 64);
    }

    else if (dato == 'M')
    {

        // MODO MANUAL

        estado_inicial = true;
        Serial.println("Modo manual");
        do
        {
            if (Serial1.available())
            {
                dato = Serial1.read();
                Serial.write(dato);
            }
            switch (dato)
            {
            // Movimiento del vehículo
            case 'a': // Modo avanzar
                if (duty == 0)
                {

                    duty = 3276;
                    porcentaje = (duty / 3276) * 10;
                    sprintf(buffer, "%d,X", porcentaje, sensor_mag);
                    Serial1.write(buffer);
                }
                roboclaw.DutyM1(address, duty);
                roboclaw.DutyM2(address, duty);
                movimiento = true;
                estado_inicial = false;
                dato = '.';
                break;
            case 'i': // Giro a la izquierda
                // roboclaw.DutyM1(address, duty);
                // roboclaw.DutyM2(address, duty*(-1));
                roboclaw.DutyM1(address, 13104);
                roboclaw.DutyM2(address, 13104 * (-1));
                estado_inicial = false;

                // Giro del 40% de duty
                // Giro del 40% de duty

                dato = '.';
                break;
            case 'd': // Giro a la derecha
                // roboclaw.DutyM1(address, duty*(-1));
                // roboclaw.DutyM2(address, duty);
                roboclaw.DutyM1(address, 13104 * (-1));
                roboclaw.DutyM2(address, 13104);
                estado_inicial = false;

                // Giro del 40% de duty
                // Giro del 40% de duty

                dato = '.';
                break;
            case 'r': // Modo reversa
                if (duty == 0)
                {
                    duty = 3276;
                    porcentaje = (duty / 3276) * 10;
                    sprintf(buffer, "%d,X", porcentaje, sensor_mag);
                    Serial1.write(buffer);
                }
                roboclaw.DutyM1(address, duty * (-1));
                roboclaw.DutyM2(address, duty * (-1));
                movimiento = false;
                estado_inicial = false;
                dato = '.';

                break;
            case 's':                        // Frenado
                roboclaw.DutyM1(address, 0); // Duty 0%
                roboclaw.DutyM2(address, 0); // Duty 0%
                estado_inicial = false;
                dato = '.';
                break;
            // Control de la velocidad del vehículo
            case '+': // Incrementar PWM de los motores
                if ((estado_inicial == true) || (duty == 0))
                {
                    roboclaw.DutyM1(address, 0);
                    roboclaw.DutyM2(address, 0);
                }
                else
                {
                    if ((duty < 29484) && (duty >= 0))
                    {
                        duty = duty + 3276;
                        porcentaje = (duty / 3276) * 10;
                        sprintf(buffer, "%d,X", porcentaje, sensor_mag);
                        Serial1.write(buffer);
                        if (movimiento == true)
                        {
                            roboclaw.DutyM1(address, duty);
                            roboclaw.DutyM2(address, duty);
                        }
                        else
                        {
                            roboclaw.DutyM1(address, duty * (-1));
                            roboclaw.DutyM2(address, duty * (-1));
                        }
                    }
                }
                delay(1000);
                dato = '.';
                break;
            case '-': // Reducir PWM de los motores
                if ((estado_inicial == true) || (duty == 0))
                {
                    roboclaw.DutyM1(address, 0);

                    roboclaw.DutyM2(address, 0);
                }
                else
                {
                    if ((duty > 0) && (duty <= 29484))
                    {
                        duty = duty - 3276;
                        porcentaje = (duty / 3276) * 10;
                        sprintf(buffer, "%d,X", porcentaje, sensor_mag);
                        Serial1.write(buffer);
                        if (movimiento == true)
                        {
                            roboclaw.DutyM1(address, duty);
                            roboclaw.DutyM2(address, duty);
                        }
                        else
                        {
                            roboclaw.DutyM1(address, duty * (-1));
                            roboclaw.DutyM2(address, duty * (-1));
                        }
                    }
                }
                delay(1000);
                dato = '.';
                break;
            // Sistema de elevación
            case 'x': // Elevación de la carga
                digitalWrite(dirPin_elevacion, HIGH);
                stop_E = false;
                while (stop_E != true)
                {
                    digitalWrite(stepPin_elevacion, HIGH);
                    delayMicroseconds(speed_E);
                    digitalWrite(stepPin_elevacion, LOW);
                    delayMicroseconds(speed_E);
                    if (digitalRead(endOfLine2) == false)
                    {
                        stop_E = true;
                    }
                }
                dato = '.';
                break;
            case 'y': // Descenso de la carga
                digitalWrite(dirPin_elevacion, LOW);
                inicio_E = false;

                while (inicio_E != true)
                {
                    digitalWrite(stepPin_elevacion, HIGH);
                    delayMicroseconds(speed_E);
                    digitalWrite(stepPin_elevacion, LOW);
                    delayMicroseconds(speed_E);
                    if (digitalRead(endOfLine1) == false)
                    {
                        inicio_E = true;
                    }
                }
                dato = '.';
                break;
            // Sistema de inserción
            case 'w': // Inserción de la carga en anaquel
                digitalWrite(dirPin_insercion, HIGH);
                inicio_IS = false;
                while (inicio_IS != true)
                {
                    digitalWrite(stepPin_insercion, HIGH);
                    delayMicroseconds(speed_IS);
                    digitalWrite(stepPin_insercion, LOW);
                    delayMicroseconds(speed_IS);
                    if (digitalRead(endOfLine3) == false)
                    {
                        inicio_IS = true;
                    }
                }
                dato = '.';
                break;
            case 'z': // Retiro de la carga desde el anaquel
                digitalWrite(dirPin_insercion, LOW);
                stop_IS = false;
                while (stop_IS != true)
                {
                    digitalWrite(stepPin_insercion, HIGH);
                    delayMicroseconds(speed_IS);
                    digitalWrite(stepPin_insercion, LOW);
                    delayMicroseconds(speed_IS);
                    if (digitalRead(endOfLine4) == false)
                    {
                        stop_IS = true;
                    }
                }
                dato = '.';
                break;
                // Sistema de sujeción

            case 'g': // El sistema de sujeción se abre
                for (posicion = 0; posicion <= 105; posicion += 1)
                {
                    servoMotorIzq.write(105 - posicion);
                    servoMotorDer.write(posicion);
                    delay(20);
                }
                dato = '.';
                break;

            case 'h': // El sistema de sujeción se cierra
                for (posicion = 105; posicion >= 0; posicion -= 1)
                {
                    servoMotorIzq.write(105 - posicion);
                    servoMotorDer.write(posicion);
                    delay(20);
                }
                dato = '.';
                break;
            case '.':
                // El sistema se detiene hasta recibir otro dato;
                break;
            case 'H':
                // Vehículo detenido
                Serial.println("Se ingresó al modo home");
                roboclaw.DutyM1(address, 0); // Duty 0%
                roboclaw.DutyM2(address, 0); // Duty 0%
                dato = 'H';
                break;
            case 'A':
                // Vehículo detenido
                Serial.println("Se ingresó al modo automático");
                roboclaw.DutyM1(address, 0); // Duty 0%
                roboclaw.DutyM2(address, 0); // Duty 0%
                dato = 'A';
                break;
            default:
                // El sistema se detiene hasta recibir otro dato;
                break;
            }

        } while ((dato != 'A') && (dato != 'H'));
    }
    else if (dato == 'H')
    {

        // ESTADO INICIAL

        estado_BT = true;
        anaquel = '0';
        // Vehículo detenido
        roboclaw.DutyM1(address, 0); // Duty 0%
        roboclaw.DutyM2(address, 0); // Duty 0%
        // Sistema de inserción retraído
        if (stop_IS != true)
        { // Si el sistema de inserción se encuentra extendido, pasa a retraerse
            digitalWrite(dirPin_insercion, LOW);
            while (stop_IS != true)
            {
                digitalWrite(stepPin_insercion, HIGH);
                delayMicroseconds(speed_IS);
                digitalWrite(stepPin_insercion, LOW);
                delayMicroseconds(speed_IS);
                if (digitalRead(endOfLine4) == false)
                {
                    stop_IS = true;
                }
            }
        }
        // Sistema de elevación en posición inferior
        if (inicio_E != true)
        { // Si el sistema de elevación se encuentra en una posición superior, pasa
            a descender
                digitalWrite(dirPin_elevacion, LOW);
            while (inicio_E != true)
            {
                digitalWrite(stepPin_elevacion, HIGH);
                delayMicroseconds(speed_E);
                digitalWrite(stepPin_elevacion, LOW);
                delayMicroseconds(speed_E);
                if (digitalRead(endOfLine1) == false)
                {
                    inicio_E = true;
                }
            }
        }
    }
}
