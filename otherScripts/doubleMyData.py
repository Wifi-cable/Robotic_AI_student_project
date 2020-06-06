#!/usr/bin/env python3
import cv2
import datetime
import os
import time
import glob
''' 
@param: string, folder with images. 
Takes all images in "imageFolder" and mirrors them. The flipped images are saved 
into the same folder. No immage name can be exidentally used twice because the 
name is tied to the unix time stamp.
@return: numbers of immages processed.
'''


# Print png images in folder C:\Users\admin\


def saveMirroredImage(imageFolder):
	counter = 0
	path = imageFolder + "/*.png"
	for img in glob.iglob(path):

		originalImage = cv2.imread(img)
		flipHorizontal = cv2.flip(originalImage, 1)
		time.sleep(1)#make sure to wait one second to prevent picture overwrite.
		dayTime = datetime.datetime.now()	#get unix time 
		now = dayTime.strftime("%s")	#save as string
		pictureName = "data" + now +".png"	#concatinate picture name from timestamp
		cv2.imwrite((imageFolder+"/" + pictureName), flipHorizontal)
		counter +=1
		#print(counter)
	return counter

def main():
	print("i flipped", saveMirroredImage("/home/wifi-cable/Semester/Semester9/Robotik/catkin_ws/misc/imgTest"), "images")

if __name__ == "__main__":
	main()
