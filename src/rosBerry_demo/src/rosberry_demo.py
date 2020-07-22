#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


#create empty message.
#all zeros mean wheels straight, no accleration
twistMessage = Twist()
twistMessage.linear.x = 0
twistMessage.linear.y = 0
twistMessage.linear.z = 0		#steering. 1 is left, 0 straight, -1 right (depends on servo)

twistMessage.angular.x = 0	#speed. can be positive or negative (to backwards)
twistMessage.angular.y = 0

speed = 0.0 #start speed
orientation = 0.0	#wheels straight 
state = 0 #start condition
countdown = 20 

Twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
#subscriber to ultrasound node
rospy.Subscriber("rangeMsg_topic", Range, driveCallback )

#method will be called whenever there is Range data
# 4 times per second
#this is the current rate of the range publisher
#Loops untill node is stopped
def driveCallback(rangeData):
	
	#speed up
	if (state == 0):
		if((rangeData <1.0) or(speed >= 2)):
			state = 1 #stop rosBerry 
	else:
		speed = speed + 0.1
	
	#slow down to do a soft stop
	if(state ==1):
		#check current speed 
		if(speed > 0):
			speed = speed -0.5
		else:
			state =2
		
	if(state == 3):
		rospy.sleep(0.25)#delay
		#backup slowly
	if(state == 4):
		orientation = 0.5
		#how log to back up for?
		if(countdown > 0):
			if(countdown %2 == 0):
				#change direction
				orientation = (orientation * -1)
			countdown = countdown - 1
		#done
		else: #not condition checks for 5. programm is done now
			state = 5
	twistMessage.linear.z = orientation
	twistMessage.angular.x = speed
	Twist_publisher.publish(twistMessage)
	

def main():
	rospy.loginfo("rosBerry Demo node started")
	node_Handle = rospy.init_node('demo_node')


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logerr("rosBerrys node crashed ")
pass

