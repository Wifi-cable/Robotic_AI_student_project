#!/usr/bin/env python
# numpy and scipy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage


def sender():
	bridge = CvBridge()
	#build a publisher
	pub = rospy.Publisher('/camera/image/compressed',CompressedImage , queue_size=0) #datatpye
	#not anonoymus means there can only be one node of this type. (only one cam)
	rospy.init_node('compress_sender', anonymous=False)
	rate = rospy.Rate(2) # 1hz

	myCam = cv2.VideoCapture(0)	#initialise camera
	
	while not rospy.is_shutdown():
		
		returnVal, capturedImg = myCam.read() #snap a picture
		rospy.loginfo("published an image")
		#can not send  directly cv images (numpy arrays) rather use cvbridge to build a ROS message
		compImg = bridge.cv2_to_compressed_imgmsg(capturedImg, dst_format='png')
		pub.publish(compImg)	#send the compressed png
		#wait to send the next image, do not try to livestream via wlan
		rate.sleep()
	rospy.logwarn("node was shut down")

if __name__ == '__main__':
	try:
		sender()
	except rospy.ROSInterruptException:
		pass
