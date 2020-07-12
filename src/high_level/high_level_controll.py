#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


DEBUG = True		#give me some output
VERBOSE = True	#tell me everything , seriouly spam me
startTime  = 0 #= rospy.get_rostime()	#not without node
updateTime = 0
instructionExecuted = False #did this node react to the last AI messgage yet?



#publisher for speed and stirring
Twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rospy.loginfo("Publisher for cmd_vel set")
#initialize variable  message with all geometric parameters of geometrymsges.Twist
message = Twist()
message.linear.x = 0
message.linear.y = 0
message.linear.z = 0

message.angular.x = 0
message.angular.y = 0


Left = "left"
Right = "right"
Forward = "forward"
NotFound = "notfound"
	
class Distance_Direction_subscriber(object):
	def __init__(self):
		if(VERBOSE):
			rospy.loginfo("constructor was called")
			
		self.direction = "NotFound"
		#more global variables?
		Twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		rospy.Subscriber('/Marker', String, self.direction_callback)		#topic datatype
		rospy.Subscriber("rangeMsg_topic", Range, self.stop_n_go_callback )
		
		
	def direction_callback(self, marker_msgs):
		self.direction = marker_msgs
		instructionExecuted = False
		if(VERBOSE):
			rospy.loginfo("direction callback got the info that %s", marker_msgs)
		
	def stop_n_go_callback(self, range_msg):
		distance = range_msg.range
		go = False
		if (DEBUG):
			rospy.loginfo( distance)
			rospy.loginfo( self.direction)
		if(distance > 0.3) : # replace if unit is not cm
			#go
			rospy.loginfo("rosBerry would drive now")
		
		else:
			
			#TODO, back up at an angle
			if(VERBOSE or DEBUG):
				rospy.loginfo("i can not drive anywhere")
				
		Twist_publisher.publish(message)
	#end stop_n_go_callback
		
	def loop(self):
		rospy.logwarn("Starting Loop to subscribe and publish")
		rospy.spin()


def main():
	rospy.loginfo("high_leve_control node started")
	node_Handle = rospy.init_node('high_leve_control')
	subscriber = Distance_Direction_subscriber()
	subscriber.loop()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logerr("try again? ")
pass
