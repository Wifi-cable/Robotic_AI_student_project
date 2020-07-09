#!/usr/bin/env python

#ROS imports
import rospy 
from std_msgs.msg import String  #
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
#python imports
import sys  
import cv2  
import enum
import numpy 
import os  
#Keras and tensorflow imports
import tensorflow as tf
from tensorflow import Graph, Session
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model

thread_graph = Graph()
with thread_graph.as_default():
	thread_session = Session()
	with thread_session.as_default():
		#model = keras.models.load_model(path_to_model)
		model = load_model('model/213x160:800@32:0.model',compile=False)
		graph = tf.get_default_graph()


VERBOSE = True #set to True for a lot of debugging output
SPAM = False	#how to you call an extra verbosity level?

class Move(enum.Enum):
	Left = "left"
	Right = "right"
	Forward = "forward"
	NotFound = "notfound"

class ImageProcessor:
	
	#python constructor
	def __init__(self):
		
		self.imgDecompresser = CvBridge()
		#build a publisher 
		self.directionPublisher = rospy.Publisher('Marker', String, queue_size=1)
		#build a subscriber
		self.sub2Img = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.imgCallBack )
		rospy.loginfo("build publiser to custom 'Maker' topic")
		rospy.loginfo("build a subscriber to '/camera/image/compressed' topic")
	
	#splits the immage in 9 tiles, seaches to seach for the marker in each
	#@param : Image from the robot camera in cv2 png fomat
	#@return : an array with likelyhoods to have found the marker for each tile
	def splitSeach(self, markerImg):
		IMG_HEIGHT, IMG_WIDTH, colors = markerImg.shape
		tiles = []
		results = []
		
		# split into 9 tiles
		step_width = IMG_WIDTH // 3
		step_height = IMG_HEIGHT // 3
		for i in range(3):
			for j in range(3):
				x_start, x_end = i * step_width, (i + 1) * step_width
				y_start, y_end = j * step_height, (j + 1) * step_height
				
				tiles.append(cv2.resize( markerImg[x_start:x_end, y_start:y_end], (step_width, step_height)))	
				#try not to mix print statments with ROS code. use  instead "rospy.loginfo"
				if(SPAM):
					rospy.loginfo("{}:{}x{}:{}".format(x_start, x_end, y_start, y_end))
		
		#threadsavety in Python is a nightmare
		with graph.as_default():
			with thread_session.as_default():
				#prediction = model.predict(data)
				for tile in tiles:
					tile = numpy.expand_dims(img_to_array( tile.astype("float") / 255.0), axis=0)
					#self.model._make_predict_function()
					(noMarker, marker) = model.predict(tile)[0]
					probability = max(noMarker, marker) * 100
					has_marker = marker > noMarker
					results.append([has_marker, probability])
			
		return results
	
	
	#callback method to do most of the work
	def imgCallBack(self, newImg):
		
		columns = []
		#compressed ROS  Image needst to be translated to opencv.
		markerImg = self.imgDecompresser.compressed_imgmsg_to_cv2(newImg)
		myResult = self.splitSeach(markerImg)
		if(VERBOSE):
			rospy.loginfo(myResult)
		
		# sort results into columns & calculate average
		for idx in range(3):
			column = [0, 0.0]  # [number of markers in column, average probability]
			for cnter in range(3):
			
				if myResult[idx + cnter* 3][0] == True:
					column[0] += 1
					column[1] += myResult[idx + cnter  * 3][1]
				
			# calculate average probability
			column[1] = column[1] /3
			columns.append(column)

		# publish where to go
		# max(columns) does sort by num of markers first, avg. probability second
		if columns[0] == max(columns) and columns[0] == [0, 0.0]:
			if(VERBOSE):
				rospy.loginfo("No marker detected anywhere.")
			self.directionPublisher.publish(Move.NotFound)
		
		elif columns[0] == max(columns):
			if(VERBOSE):
				rospy.loginfo("Go left, I'm {}% sure".format(columns[0][1]))
			self.directionPublisher.publish(Move.Left)
		
		elif columns[1] == max(columns):
			if(VERBOSE):
				rospy.loginfo("Go forward, I'm {}% sure".format(columns[1][1]))
			self.directionPublisher.publish(Move.Forward)
		
		elif columns[2] == max(columns):
			if(VERBOSE):
				rospy.loginfo("Go right, I'm {}% sure".format(columns[2][1]))
			self.directionPublisher.publish(Move.Right)

		else:
			rospy.logerr("confused AI, the marker is not here, nor is there no marker")



def main():
	'''Initializes and cleanup ros node'''
	ic = ImageProcessor()
	rospy.loginfo("create AI node")
	rospy.init_node('AI')
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.logerr("rospy spin failed, your node crashed")


if __name__ == '__main__':
	main()
