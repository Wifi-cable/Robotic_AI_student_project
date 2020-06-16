#!/usr/bin/env python3
import cv2	#opencv
import sys	#fileIO stuff
import os		#other fileIO stuff
import numpy	#for opencv
import rospy	#need Ros Python
from std_msgs.msg import String # ROS python needs a Ros message to publish

'''
TODO
- take picture
- use opencv to load picture in variable
- cut picture in 9 pices
- deploy model of Neural Net
- seach for markler in pices (or in whole picture first)
- figure out if marker is in left, right, center or nix. 
- publish result as string

'''
pub = rospy.Publisher('Marker', String, queue_size=10)	#instead of return you publish
rospy.init_node('find_marker')	# naming a node makes a lot of sense for debugging. choose any name.
myRate = rospy.Rate(10) # 10hz	#10 may need adjustment 
while not rospy.is_shutdown():	#not while true,  it is while not ctrl+C from terminal.
   pub.publish("hello world")	#replace by String message to tell us where the marker is
   myRate.sleep()		#like delay. to not spam my terminal 100 times per second 
