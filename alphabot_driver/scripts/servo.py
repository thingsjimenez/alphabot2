#!/usr/bin/env python

import rospy
import roslib
import tf
from alphabot_msgs.msg import pan_tilt

import time
import math
import smbus

#-- Docs SMBus
# http://wiki.erazor-zone.de/wiki:linux:python:smbus:doc
# https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf

class PCA9685:
	# Registers/etc.
	__SUBADR1	= 0x02
	__SUBADR2	= 0x03
	__SUBADR3	= 0X04
	__MODE1		= 0x00
	__PRESCALE	= 0xFE
	__LED0_ON_L	= 0x06
	__LED0_ON_H	= 0x07
	__LED0_OFF_L	= 0x08
        __LED0_OFF_H	= 0x09
	__ALLLED_ON_L	= 0xFA
        __ALLLED_ON_H	= 0xFB
        __ALLLED_OFF_L	= 0xFC
        __ALLLED_OFF_H	= 0xFD

	def __init__(self, address=0x40, debug=False):
		rospy.loginfo("Node 'Robot RR' configuring PCA9685")
		self.bus = smbus.SMBus(1)
		self.address = address
		self.debug = debug
		rospy.loginfo("Node 'Robot RR' Reseting PCA9685")
		self.write(self.__MODE1, 0x00)
		rospy.loginfo("Node 'Robot RR' PCA9685 configured")

	def write(self, reg, value):
		self.bus.write_byte_data(self.address, reg, value)

	def read(self, reg):
		result = self.bus.read_byte_data(self.address, reg)
		return result

	def setPWMFreq(self, freq):
		prescaleval = 25000000.0	# 25MHz
		prescaleval /= 4096.0		# 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
		prescale = math.floor(prescaleval + 0.5)

		oldmode = self.read(self.__MODE1)
		newmode = (oldmode & 0x7F) | 0x10
		self.write(self.__MODE1, newmode)
		self.write(self.__PRESCALE, int(math.floor(prescale)))
		self.write(self.__MODE1, oldmode)
		time.sleep(0.005)
		self.write(self.__MODE1, oldmode | 0x80)

	def setPWM(self, channel, on, off):
		self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
		self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
		self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
                self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

	def setServoPulse(self, channel, pulse):
		pulse = pulse * 4096/20000
		self.setPWM(channel, 0, pulse)

	def __del__(self):
		self.write(self.__MODE1, 0x00)

class pan_tilt_driver:
	# Set HPulse & VPulse to half way {500 to 2500}
	HPulse = 1500
	VPulse = 1500
	pan = 0
	tilt = 0
	pan_offset = 0
	tilt_offset = 0
	pan_center_pulse = 1500
	tilt_center_pulse = 1500
	pan_limit_left = 1.5
	pan_limit_right = 1.5
	tilt_limit_up = 1.5
	tilt_limit_down = 0.8

	def __init__(self):
		rospy.init_node("pan_tilt_driver")
		rospy.loginfo("Node 'Robot RR' configuring driver")

		self.rate = rospy.Rate(rospy.get_param('~rate', 10))

		self.pwm = PCA9685(0x40)
		self.pwm.setPWMFreq(50)

		self.pan_offset = rospy.get_param('~pan_offset', self.pan_offset)
		self.tilt_offset = rospy.get_param('~tilt_offset', self.tilt_offset)
		self.pan_limit_left = rospy.get_param('~pan_limit_left', self.pan_limit_left)
		self.pan_limit_right = rospy.get_param('~pan_limit_right', self.pan_limit_right)
		self.tilt_limit_up = rospy.get_param('~tilt_limit_up', self.tilt_limit_up)
		self.tilt_lmit_down = rospy.get_param('~tilt_limit_down', self.tilt_limit_down)

		rospy.loginfo("Node 'Robot RR' pan_offset = "+str(self.pan_offset)+".")
		rospy.loginfo("Node 'Robot RR' tilt_offset = "+str(self.tilt_offset)+".")
		rospy.loginfo("Node 'Robot RR' pan_limit_left = "+str(self.pan_limit_left)+".")
		rospy.loginfo("Node 'Robot RR' pan_limit_right = "+str(self.pan_limit_right)+".")
		rospy.loginfo("Node 'Robot RR' tilt_limit_up = "+str(self.tilt_limit_up)+".")
		rospy.loginfo("Node 'Robot RR' tilt_lmit_down = "+str(self.tilt_lmit_down)+".")

		# Center servos
		self.pan_center_pulse = int(-self.pan_offset  * (1000 / (math.pi / 2)) + self.pan_center_pulse)
		self.tilt_center_pulse = int(-self.tilt_offset * (1000 / (math.pi / 2)) + self.tilt_center_pulse)

		self.HPulse = self.pan_center_pulse
		self.VPulse = self.tilt_center_pulse

		#Set the Horizontal servo parameters
		self.pwm.setServoPulse(0, self.HPulse)

		#Set the vertical servo parameters
		self.pwm.setServoPulse(1, self.VPulse)

		# Setup subscriber for pan_tilt message
		rospy.Subscriber('~pan_tilt', pan_tilt, self.pan_tilt_callback)
		rospy.loginfo("Node 'Robot RR' configured")

	def run(self):
		rospy.loginfo("Node 'pan_tilt' running.")

		# Create TransformBroadcastrer
		br = tf.TransformBroadcaster()

		while not rospy.is_shutdown():
			# Send Pan tf
			br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0, self.pan), rospy.Time.now(), "servoPan", "servoPanNeck")
			# Send Tilt tf
			br.sendTransform((0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(0.0, self.tilt, 0.0), rospy.Time.now(), "servoTilt", "servoPan")
			self.rate.sleep()

	def pan_tilt_callback(self, message):
		rospy.loginfo("Node 'camera_pan_tilt' message received.")
		# limit the servo pan
		if message.pan >=0:
			self.pan = min([message.pan, self.pan_limit_left])
		else:
			self.pan = max([message.pan, -self.pan_limit_right])

		# Limit the servo tilt
		if message.tilt >=0:
    			self.tilt = min([message.tilt, self.tilt_limit_up])
		else:
			self.tilt = max([message.tilt, -self.tilt_limit_down])

		rospy.loginfo("Node 'Robot RR' Pan= "+str(self.pan)+".")
		rospy.loginfo("Node 'Robot RR' Tilt="+str(self.tilt)+".")

		# Convert from radians to pulses (500 - 2500)
		self.HPulse = int(self.pan  * (1000 / (math.pi / 2)) + self.pan_center_pulse)
		self.VPulse = int(self.tilt * (1000 / (math.pi / 2)) + self.tilt_center_pulse)
		self.pwm.setServoPulse(1, self.VPulse)
		self.pwm.setServoPulse(0, self.HPulse)

def main():
	rospy.loginfo("Starting node 'pan_tilt'")
	driver = pan_tilt_driver()
	driver.run()
	rospy.loginfo("Node 'pan_tilt' Stopped")

if __name__ == '__main__':
	main()


