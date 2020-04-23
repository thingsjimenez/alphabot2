#!/usr/bin/env python

import rospy
from motor_node import MotorDriver

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class MotorDriverWrapper:
	def __init__(self):
		rospy.init_node('driver_motors')
		self._last_received = rospy.get_time()
		# Se toman los parametros del servidor de parametros del master
		self._timeout = rospy.get_param('~timeout', 2)
		self._rate = rospy.get_param('~rate', 10)
		self._max_speed = rospy.get_param('~max_speed', 0.25)
		self._wheel_base = rospy.get_param('~wheel_base', 0.093)
		# Asignacion de los pines del motor.
		# Depende de la configuracion del robot
		self._left_motor = MotorDriver(13, 12, 6)
		self._right_motor = MotorDriver(21, 20, 26)
		self._left_speed_percent = 0
		self._right_speed_percent = 0
		# Se suscribe a los comandos de velocidad y se crean los servicios
		rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)
		rospy.Service("stop_motors", Trigger, self.callback_stop)
		
	def velocity_received_callback(self, message):
		"""Tratamiento de los mensajes de velocidad."""
		self._last_received = rospy.get_time()
		# Se extrae la velocidad lineal y angular del mensaje
		linear = message.linear.x
		angular = message.angular.z
		# Calculo de la velocidad de las llantas en m/s
		left_speed = linear - angular * self._wheel_base/2
		right_speed = linear + angular * self._wheel_base/2
		# Se deberia utilizar la velocidad del sensor del motor pero como no se cuenta
		# con un sensor. Simplemente se convierte la velocidad en m/s en porcentaje de
		# la velocidad maxima de la rueda, a un ciclo de trabajo que se pueda aplicar
		# a cada motor.
		self._left_speed_percent = (100 * left_speed/self._max_speed)
		self._right_speed_percent = (100 * right_speed/self._max_speed)
    
	def callback_stop(self, req):
		self._left_motor.stop
		self._right_motor.stop
		rospy.loginfo('Los motores se han parado')

	def stop(self):
		self._left_motor.stop
		self._right_motor.stop

	def run(self):
		"""Lazo de control del driver."""
		rate = rospy.Rate(self._rate)
		while not rospy.is_shutdown():
			# Si no se reciben comandos, o se pierde
			# la conexion para parar los motores
			delay = rospy.get_time() - self._last_received
			if delay < self._timeout:
				self._left_motor.move(self._left_speed_percent)
				self._right_motor.move(self._right_speed_percent)
			else:
				rospy.logwarn('El delay es superior al timeout: %f', self._timeout)
				self._left_motor.move(0)
				self._right_motor.move(0)
			rate.sleep()

def main():
	driver = MotorDriverWrapper()
	rospy.loginfo('El controlador de motores ya está en marcha, listo para recibir comandos')
	driver.run()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logerr('Error!!')
