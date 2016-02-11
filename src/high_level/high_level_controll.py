#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


DEBUG = True		#give me some output
VERBOSE = True	#tell me everything , seriouly spam me

instructionExecuted = False #did this node react to the last AI messgage yet?

#publisher for speed and stirring
Twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
rospy.loginfo("Publisher for cmd_vel set")
#initialize variable  message with all geometric parameters of geometrymsges.Twist
message = Twist()
message.linear.x = 0
message.linear.y = 0
message.linear.z = 0

message.angular.x = 0
message.angular.y = 0

	
class Distance_Direction_subscriber(object):
	def __init__(self):
		if(VERBOSE):
			rospy.loginfo("constructor was called")
			
		self.direction = "initVal"
		#more global variables?
		Twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		rospy.Subscriber('/Marker', String, self.direction_callback)		#topic datatype
		rospy.Subscriber("rangeMsg_topic", Range, self.stop_n_go_callback )
		
	def direction_callback(self, marker_msgs):
		self.direction = marker_msgs.data.upper()
		instructionExecuted = False
		if(VERBOSE):
			rospy.loginfo("direction callback got the info: " +self.direction)
		
	def stop_n_go_callback(self, range_msg):
		distance = range_msg.range
		orientation = self.direction
		
		
		if (DEBUG):
			rospy.loginfo( distance)
			#rospy.loginfo(self.direction)
			
		if(distance > 0.3 and (orientation != 'NOTFOUND')): # replace if unit is not 
			message.linear.x = 0.0
			if(VERBOSE):
				rospy.loginfo("rosBerry would drive "+ orientation)
				
			
			if(orientation =='LEFT'):
				message.angular.z = 1
				message.linear.x = 0.4
				
			elif(orientation == 'RIGHT'):
				message.angular.z = -1
				message.linear.x = 0.4
			
			elif(orientation == 'FORWARD'):
				message.angular.z = 0
				message.linear.x = 0.4
			
			#this should not happen. whatever it is, stop the robot
			else:
				rospy.logerr("rosBerry can not go "+orientation)
				message.linear.x = 0.0
				
			#about to hit a wall
		else:
			if(VERBOSE or DEBUG):
				rospy.loginfo("I can not drive anywhere, I will back away slowly")
				
			#roll backwarts slowly at an angle before you hit a wall
			message.angular.z = -0.2
			message.linear.x = -0.4
			
		executed = True
		Twist_publisher.publish(message)
	#end stop_n_go_callback 			#TODO, back up at an angle
		
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
