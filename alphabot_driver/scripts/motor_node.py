#!/usr/bin/env python

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
_FREQUENCY = 22000

class MotorDriver:
	def __init__(self, forward_pin, backward_pin, enable_pin):
		# Configuracion	de pines
		self._forward_pin = forward_pin
		self._backward_pin = backward_pin
		self._enable_pin = enable_pin
		# Configuracion de estado de pines del RPi
		GPIO.setup(self._forward_pin, GPIO.OUT)
		GPIO.setup(self._backward_pin, GPIO.OUT)
		GPIO.setup(self._enable_pin, GPIO.OUT)
		GPIO.output(self._enable_pin, GPIO.HIGH)
		# Configuracion de PWM
		self._forward_pwm = GPIO.PWM(self._forward_pin, _FREQUENCY)
		self._backward_pwm = GPIO.PWM(self._backward_pin, _FREQUENCY)
	
	def _clip(self, value, minimum, maximum):
		"""El valor se limita entre el valor minimo y maximo."""
		if value < minimum:
			return minimum
		elif value > maximum:
			return maximum
		return value
	
	def move(self, speed_percent):
		"""A velocidades positivas el movimiento es hacia adelante """
		""" y a velocidades negativas el movimiento es hacia atras."""
		speed = self._clip(abs(speed_percent), 0, 100)
		if speed_percent < 0:
			self._backward_pwm.start(speed)
			self._forward_pwm.start(0)
		else:
			self._forward_pwm.start(speed)
			self._backward_pwm.start(0)

	def stop(self):
		"""Se para la salida del PWM."""
		self._forward_pwm.start(0)
		self._backward_pwm.start(0)


