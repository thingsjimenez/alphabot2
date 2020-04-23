#!/usr/bin/env python

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class JoyDriver:
	def __init__(self, CTR, A, B, C, D):
		self._CTR = CTR
		self._A = A
		self._B = B
		self._C = C
		self._D = D
		GPIO.setup(self._CTR, GPIO.IN, GPIO.PUD_UP)
		GPIO.setup(self._A, GPIO.IN, GPIO.PUD_UP)
		GPIO.setup(self._B, GPIO.IN, GPIO.PUD_UP)
		GPIO.setup(self._C, GPIO.IN, GPIO.PUD_UP)
		GPIO.setup(self._D, GPIO.IN, GPIO.PUD_UP)

	def status_CTR(self):
		return GPIO.input(self._CTR)

	def status_A(self):
		return GPIO.input(self._A)

	def status_B(self):
		return GPIO.input(self._B)

	def status_C(self):
		return GPIO.input(self._C)

	def status_D(self):
		return GPIO.input(self._D)

	
