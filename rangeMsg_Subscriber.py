#!/usr/bin/env python
# used tab 4

import rospy
import math
from sensor_msgs.msg import Range

#Float
global meters
global centimeters

def callback(data):
	rospy.loginfo(rospy.get_name() + " I heard %f", data.range)
	meters = data.range
	rospy.loginfo("%f meters", meters)
	printCentimeters(meters)

def printCentimeters(value):
	centimeters = transformMetersToCentimeters(value)
	print str(centimeters) + " cm"

# meters to centimeters with 2 decimal places
def transformMetersToCentimeters(value):
	return round(value * 100, 2)

def ultrasoundListener():
	rospy.init_node('ultrasound_listener', anonymous=True)
	rospy.Subscriber("rangeMsg_topic", Range, callback)
	rospy.spin()

if __name__ == '__main__':
	ultrasoundListener()
