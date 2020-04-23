#!/usr/bin/env python

import rospy
from inf_avoid import InfraredAvoid
from alphabot2_msgs.msg import sensorArray

DIODERIGHT = 16
DIODELEFT = 19

def talker():
	rospy.init_node('infrared_avoid', anonymous=False)
	rospy.loginfo('Inicio de los sensores')
	IR_diodes = InfraredAvoid(DIODERIGHT, DIODELEFT)
	pub = rospy.Publisher('infrared_diodes', sensorArray, queue_size=3)
	rate = rospy.Rate(4)
	msgArray = sensorArray()
	while not rospy.is_shutdown():
		msgArray.header.seq += 1
		msgArray.header.stamp = rospy.Time.now()
		msgArray.header.frame_id = 'infrared_avoid'
		msgArray.data = [IR_diodes.status_R(), IR_diodes.status_L()]
		pub.publish(msgArray)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		rospy.logerr('Error!!')
