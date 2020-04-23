#!/usr/bin/env python

import rospy
from joy_node import JoyDriver
from alphabot2_msgs.msg import sensorArray

CTR = 7
A = 8
B = 9
C = 10
D = 11

def talker():
	rospy.init_node('joy_node', anonymous=False)
	rospy.loginfo('Inicio de los sensores')
	j_node = InfraredAvoid(CTR, A, B, C, D)
	pub = rospy.Publisher('joy_cmd', sensorArray, queue_size=3)
	rate = rospy.Rate(4)
	msgArray = sensorArray()
	while not rospy.is_shutdown():
		msgArray.header.seq += 1
		msgArray.header.stamp = rospy.Time.now()
		msgArray.header.frame_id = 'joy_node'
		msgArray.data = [j_node.status_CTR(), j_node.status_A(), j_node.status_B(), j_node.status_C(), j_node.status_D()]
		pub.publish(msgArray)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		rospy.logerr('Error!!')
