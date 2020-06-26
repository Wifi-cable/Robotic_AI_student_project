#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

#Twist_publisher = rospy.Publisher('/dkcar/control/cmd_vel', Twist, queue_size=2)
Twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rospy.loginfo("Publisher for cmd_vel set")
message = Twist()
message.linear.x = 0
message.linear.y = 0
message.linear.z = 0

message.angular.x = 0
message.angular.y = 0


def callback(my_marker):

	string_in = my_marker.data
	#rospy.loginfo("I got %s from the Marker topic",string_in)	
	string_in = string_in.upper()	#spelling is a pain in the ___ make input string upper case
	if(string_in == 'LEFT'):
		rospy.loginfo("RosbBerry would go left now")
		message.angular.z = 1

	elif(string_in == 'RIGHT'):
		rospy.loginfo("RosbBerry would go right now")
		message.angular.z = -1

	elif(string_in == 'STRAIGHT'):
		rospy.loginfo("RosbBerry would go straight now")
		message.angular.z = 0
	elif(string_in == 'STOP'):
		rospy.loginfo("RosbBerry would do a savety stop now")
		speed = 0.0
	else:
		rospy.loginfo("what? %s Rosberry is very confused now", string_in)

	Twist_publisher.publish(message)



#rostopic pub -1 /marker_topic std_msgs/String  left


def main():
	rospy.loginfo("high_leve_control node started")
	node_Handle = rospy.init_node('high_leve_control')
	subscriber = rospy.Subscriber("marker_topic", String, callback)


	myRate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		#pub.publish(message)
		myRate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logerr("try again? ")
pass
