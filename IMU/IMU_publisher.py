#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool, UInt32, Int32, UInt16MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import serial
import codecs
from math import *
import numpy as np
import os


os.chdir("/home/tumi/InspeccionPlantas/inspection_ws/src/auto_nav/src/scripts")
flagINS = False


P_muestreo= 100 #ms
nC = [0,0,0,0]


''' ==============================
                 CLASS
    =============================='''
class ImuSensor:
	roll = 0
	pitch = 0
	yaw = 0
	acceX = 0
	acceY = 0
	acceZ = 0
	gyroX = 0
	gyroY = 0
	gyroZ = 0
	q0 = 0
	q1 = 0
	q2 = 0
	q3 = 0


'''================================
           FUNCTIONS
===================================
'''
def publishStates():


	# Publishing imu data
	msg = Imu()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "inertial_sense"
	msg.orientation.w = imu.q0
	msg.orientation.x = imu.q1
	msg.orientation.y = imu.q2
	msg.orientation.z = imu.q3
	msg.angular_velocity.x = imu.gyroX
	msg.angular_velocity.y = imu.gyroY
	msg.angular_velocity.z = imu.gyroZ
	msg.linear_acceleration.x = imu.acceX
	msg.linear_acceleration.y = imu.acceY
	msg.linear_acceleration.z = imu.acceZ
	imu_pub.publish(msg)

	msg = Vector3()
	msg.x = imu.roll
	msg.y = imu.pitch
	msg.z = imu.yaw
	euler_pub.publish(msg)

	

def read_file(filename):
    try:
        with open(filename, 'r') as file:
            data = file.readline().strip()  # Leer una línea del archivo y eliminar espacios en blanco
            return data.split(', ')  # Dividir la cadena en una lista usando ', ' como separador
    except FileNotFoundError:
        print(f"El archivo {filename} no se encontró.")
        return None


	
#########################################################################
#########################################################################

imu = ImuSensor()
filename_ins = "/home/tumi/InspeccionPlantas/inspection_ws/src/auto_nav/src/scripts/ins1.txt"
filename_pimu = "/home/tumi/InspeccionPlantas/inspection_ws/src/auto_nav/src/scripts/pimu.txt"

if __name__ == '__main__':

	try:
#		rospy.init_node('main_node', anonymous=True)
		rospy.init_node('imu_node', log_level=rospy.DEBUG)
		rospy.logdebug("DEBUG MODE enabled")

		imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
		euler_pub = rospy.Publisher('/imu/data_euler', Vector3, queue_size=10)
		
		rate = rospy.Rate(10)

		rospy.loginfo("Main node initialized!!")

		while not rospy.is_shutdown():
			
			ins_data = read_file(filename_ins)
			if ins_data:
				try:
					rospy.logdebug("INS comunication")
					roll,pitch,yaw= [float(value.split(': ')[1]) for value in ins_data[:3]]
					imu.roll = roll
					imu.pitch = pitch
					imu.yaw = yaw

					global _FLAG_CONECT
					self.act3_button.setEnabled(True)
					self.act4_button.setEnabled(True)

					if not _FLAG_CONECT:

						# Crear un socket TCP/IP
						_VAR_socketB = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

						# Conectar el socket al puerto donde el servidor está escuchando
						server_address = ('192.168.1.78', PORT_B)
						print(
							f"Conectando a {server_address[0]} puerto {server_address[1]}")
						_VAR_socketB.connect(server_address)
						_FLAG_CONECT = True

					try:
						# Enviar datos
						message = f'$OAX2JA'
						print(f"Enviando: {message}")
						_VAR_socketB.sendall(message.encode('utf-8'))
						_FLAG_CONECT = False

					except:
						print("Cerrando socket")
					rospy.logdebug("roll = %f :: pitch = %f :: yaw = %f" % (imu.roll, imu.pitch, imu.yaw))
					#print(f"INS - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
				except Exception as e:
					rospy.logerr(e)
					pass
			
			pimu_data = read_file(filename_pimu)
			if pimu_data:
				try:

					rospy.logdebug("PIMU comunication")
					gyro_x, gyro_y, gyro_z = [float(value.split(': ')[1]) for value in pimu_data[:3]]
					acc_x, acc_y, acc_z = [float(value.split(': ')[1]) for value in pimu_data[3:]]
					imu.gyroX = gyro_x
					imu.gyroY = gyro_y
					imu.gyroZ = gyro_z
					imu.acceX = acc_x
					imu.acceY = acc_y
					imu.acceZ = acc_z
					rospy.logdebug("gyroX = %f :: gyroY = %f :: gyroZ = %f" % (imu.gyroX, imu.gyroY, imu.gyroZ))
					rospy.logdebug("acceX = %f :: acceY = %f :: acceZ = %f" % (imu.acceX, imu.acceY, imu.acceZ))
					#print(f"PIMU - GyroX: {gyro_x}, GyroY: {gyro_y}, GyroZ: {gyro_z}, AccX: {acc_x}, AccY: {acc_y}, AccZ: {acc_z}")
				
				except Exception as e:
					rospy.logerr(e)
					pass
			
			publishStates()

			rate.sleep()

		
	except Exception as e:
					rospy.logerr(e)
					pass
