#!/usr/bin/env python

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


class InfraredAvoid:
	def __init__(self, DR, DL):
		self._diode_right = DR
		self._diode_left = DL
		GPIO.setup(self._diode_right, GPIO.IN, GPIO.PUD_UP)
		GPIO.setup(self._diode_left, GPIO.IN, GPIO.PUD_UP)
	
	def status_R(self):
		return GPIO.input(self._diode_right)

	def status_L(self):
		return GPIO.input(self._diode_left)

