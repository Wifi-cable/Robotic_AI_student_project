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
		path = os.path.dirname(os.path.abspath(__file__)) + '/../model/213x160:800@32:0.model'
		model = load_model(path,compile=False)
		graph = tf.get_default_graph()


VERBOSE = True #set to True for a lot of debugging output
SPAM = False	#how to you call an extra verbosity level?


Left = "left"
Right = "right"
Forward = "forward"
NotFound = "notfound"
HalfRight = "halfright"
HalfLeft = "halfleft"



class ImageProcessor:
	
	#python constructor
	def __init__(self):
		self.counter = 0
		self.imgDecompresser = CvBridge()
		#build a publisher 
		self.directionPublisher = rospy.Publisher('/Marker', String, queue_size=1)
		#build a subscriber
		self.sub2Img = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.imgCallBack )
		#self.sub2Img = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.highscoreCallback )

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
		
		if(VERBOSE):
			now = str(rospy.Time.now)
			cv2.imshow("stream ",markerImg)
			cv2.waitKey(3)
			#rospy.sleep(5.5)
			#cv2.destroyAllWindows()
		
		if(SPAM):
			rospy.loginfo("IMG: {}x{}".format(round(IMG_WIDTH,2), round(IMG_HEIGHT,2)))
			cv2.imshow("Livestream ",markerImg)
			rospy.sleep(0.5)
			cv2.destroyAllWindows()
			
		for i in range(3):
			for j in range(3):
				x_start, x_end = i * step_width, (i + 1) * step_width
				y_start, y_end = j * step_height, (j + 1) * step_height
				
				tiles.append(cv2.resize( markerImg[x_start:x_end, y_start:y_end], (step_width, step_height)))	
				#try not to mix print statments with ROS code. use  instead "rospy.loginfo"
				if(SPAM):
					rospy.loginfo("{}:{}x{}:{}".format(round(x_start,2), round(x_end,2), round(y_start,2), round( y_end,2)))
		
		#threadsavety in Python is a nightmare
		with graph.as_default():
			with thread_session.as_default():
				for tile in tiles:
				
					tile = numpy.expand_dims(img_to_array( tile.astype("float") / 255.0), axis=0)
					(noMarker, marker) = model.predict(tile)[0]
					probability = round((max(noMarker, marker) * 100),2)
					has_marker = marker > noMarker
					results.append([has_marker, probability])
			
		return results
	
	
	#callback method to do most of the work
	def imgCallBack(self, newImg):
		now = rospy.Time.now()
		send_time = newImg.header.stamp
		img_delay = now.secs - send_time.secs
		im_del =  now.nsecs - send_time.nsecs
		global VERBOSE 
		global SPAM 
		
		if(self.counter >= 3):
			VERBOSE = False
			SPAM =  False
			#cv2.destroyAllWindows()
			#rospy.logwarn("OFFFFFFFFF!!!!!!")
		self.counter = self.counter + 1
		#rospy.logwarn(self.counter)
		if(VERBOSE):
			rospy.logwarn("Transfer Delay: {}.{} Sec".format(img_delay, im_del, "\n"))
		
		#columns = numpy.array()
		columns = []
		#compressed ROS  Image needst to be translated to opencv.
		markerImg = self.imgDecompresser.compressed_imgmsg_to_cv2(newImg)
		myResult = self.splitSeach(markerImg)
		if(VERBOSE):
			#rospy.loginfo(round(myResult,2))
			rospy.loginfo(myResult)
			#rospy.loginfo("{:.2f}".format(myResult))
		
		# sort results into columns & calculate average
		for idx in range(3):
			column = [0, 0.0]  # [number of markers in column, average probability]
			for cnter in range(3):
			
				if myResult[(idx * 3) + cnter][0] == True:
					column[0] += 1
					column[1] += myResult[(idx * 3) + cnter][1]
		
			# calculate average probability
			#column[1] = column[1] /3
			columns.append(column)
		# publish where to go
		if(VERBOSE):
			rospy.loginfo('')
			rospy.loginfo('Oben.: {:5}:{:6.2f}  {:5}:{:6.2f}  {:5}:{:6.2f}'.format(str(myResult[0][0]), myResult[0][1] , str(myResult[3][0]), myResult[3][1], str(myResult[6][0]), myResult[6][1]))
			rospy.loginfo('Mitte: {:5}:{:6.2f}  {:5}:{:6.2f}  {:5}:{:6.2f}'.format(str(myResult[1][0]), myResult[1][1] , str(myResult[4][0]), myResult[4][1], str(myResult[7][0]), myResult[7][1]))
			rospy.loginfo('Unten: {:5}:{:6.2f}  {:5}:{:6.2f}  {:5}:{:6.2f}'.format(str(myResult[2][0]), myResult[2][1] , str(myResult[5][0]), myResult[5][1], str(myResult[8][0]), myResult[8][1]))
			rospy.loginfo('')
			
			
		if(VERBOSE):
			rospy.loginfo('Summe: \033[33m<<--{:1}:{:6.2f}  \033[34m^^^^{:1}:{:6.2f}  \033[1;34m-->>{:1}:{:6.2f}\033[0m'.format(columns[0][0],columns[0][1], columns[1][0],columns[1][1], columns[2][0],columns[2][1]))
			rospy.loginfo("shape of the columns arraylist is: ")
			rospy.loginfo(columns)
			rospy.loginfo(max(columns))
			#rospy.loginfo(columns[0])
		
		#if ((columns[0] == max(columns)) and (columns[0] == [0, 0.0])):
		if ((columns[0] == [0, 0.0]) and (columns[1] == [0, 0.0]) and (columns[2] == [0, 0.0])):
			if(VERBOSE):
				rospy.loginfo("---------------------------- No marker detected anywhere.")
			self.directionPublisher.publish(NotFound)
			
			#if unsure where the Marker is, threat situation like marker not found
		certainty = 30 # threashold of certainty where the marker is
		if((columns[0][1] < certainty) and (columns[1][1] < certainty) and (columns[2][1] < certainty)):
			if(VERBOSE or SPAM):
				rospy.loginfo("-----------------------------not sure where the marker is")
			self.directionPublisher.publish(NotFound)
			
			#marker is somewhat left and ahead
		elif((columns[0] != [0, 0.0]) and (columns[1] != [0, 0.0])): 
			if(VERBOSE):
				certainty = (( columns[0][1] + columns[1][1]) /2)
				rospy.loginfo("-------------- Go  a little left, I'm {}% sure".format(certainty,2))
			self.directionPublisher.publish(HalfLeft)
			
		#marker is not quite ahead more a little right
		elif((columns[1] != [0, 0.0]) and (columns[2] != [0, 0.0])):
			if(VERBOSE):
				certainty = (( columns[1][1] + columns[2][1]) /2)
				rospy.loginfo("-------------- Go  a little right, I'm {}% sure".format(round(certainty,2)))
			self.directionPublisher.publish(HalfRight)
		
		elif columns[0] == max(columns):
			if(VERBOSE):
			# round(myfloat,2) to get a float with two digits after the comma
				rospy.loginfo("------------------ Go left, I'm {}% sure".format(round(columns[0][1],2)))
			self.directionPublisher.publish(Left)
		
		elif columns[1] == max(columns):
			if(VERBOSE):
				rospy.loginfo("---------------- Go forward, I'm {}% sure".format(round(columns[1][1],2)))
			self.directionPublisher.publish(Forward)
		
		elif columns[2] == max(columns):
			if(VERBOSE):
				rospy.loginfo("-------------- Go right, I'm {}% sure".format(round (columns[2][1],2)))
			self.directionPublisher.publish(Right)
			
		else:
			rospy.logerr("--------------- confused AI, the marker is not here, nor is there no marker")
			
		
		
		
	def highscoreCallback(self, newImg):
		now = rospy.Time.now()
		send_time = newImg.header.stamp
		img_delay = now.secs - send_time.secs
		im_del =  now.nsecs - send_time.nsecs
		
		if(VERBOSE):
			rospy.logwarn("Transfer Delay: {}.{} Sec".format(img_delay, im_del, "\n"))
		
		markerImg = self.imgDecompresser.compressed_imgmsg_to_cv2(newImg)
		myResult = self.splitSeach(markerImg)
		
		if(VERBOSE):
			rospy.loginfo(myResult)
		#have [[t/f, %]
		#can not be numpy array because it has mixed data types 
		#can have two arrays one for true and one for false, 
		#or fase means  (-1 * x)
		#add all numbers per column up.
		
		'''
		for idx in range(3):
			column = [0, 0.0]  # [number of markers in column, average probability]
			for cnter in range(3):
			
				if myResult[(idx * 3) + cnter][0] == True:
					column[0] += 1
					column[1] += myResult[(idx * 3) + cnter][1]
		'''
		
		#the myResults array is nested, one dimentional.[ [bool, float] [bool,float]...
		#this loop looks at 3 columns if the array where 3D
		#idx:  0, 3, 6, | 1, 4, 7| 2, 5, 8
		acumulator_array = numpy.zeros((3,3))
		for col in range(3):
			for row in range(3):
				idx = row +(col *3)
				#if marker detection is false, make value negative
				if(myResult[idx][0] == False):
					#acumulator_array[row][col] = ( -1 * (round( myResult[idx][1]),2))	#out of range
					
					acumulator_array[row][col] =  (-1 * idx)
				else:
					#acumulator_array[row][col] = (round( myResult[idx][1]), 2)	#out of range
					
					acumulator_array[row][col] = idx #debug
		if(VERBOSE):
			rospy.loginfo(acumulator_array)


def main():
	'''Initializes and cleanup ros node'''
	ic = ImageProcessor()
	rospy.loginfo("create AI node")
	rospy.init_node('AI')
	try:
		rospy.spin()

	except KeyboardInterrupt:
		rospy.logerr("rospy spin failed, your node crashed")
	rospy.logwarn("find_publish_marker node was shut down") 

if __name__ == '__main__':
	main()
