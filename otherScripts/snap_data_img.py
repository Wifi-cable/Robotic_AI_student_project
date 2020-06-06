#!/usr/bin/env python3
import cv2
import datetime
import sys
import argparse
import os
import numpy

def undistort(imgName):
	img = cv2.imread(imgName)
	#    filename = workingFolder + "/cameraMatrix.txt"
	#    np.savetxt(filename, mtx, delimiter=',')
	#    filename = workingFolder + "/cameraDistortion.txt"
	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
	h,  w = img.shape[:2]
	newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
	
	# undistort
	dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
	
	x,y,w,h = roi	#cropping
	dst = dst[y:y+h, x:x+w]
	cv2.imwrite('calibresult.png',dst)
	return 0#dummy value

def snapPicture(marker=False, show=False):
	print("marker= "+str(marker)+ " show = "+ str(show))
	dayTime = datetime.datetime.now()	#get unix time of when picture was taken
	now = dayTime.strftime("%s")	#save as string
	cap = cv2.VideoCapture(0)
	ret, capture = cap.read()
	pictureName = "data" + now +".png"
	print(pictureName)
#if immage captured correctly save it.
	print("taking a picture was sucessfull? " + str(ret))
	
	if ret:	
		if marker:     #save in folder with marker
			folder = "marker/"
			cv2.imwrite((folder + pictureName), capture)
		else:    #save in other folder
			folder = "no_marker/"
			cv2.imwrite((folder + pictureName), capture)
		print("saved a picture named "+ pictureName +" to the folder " + folder) 
		
		#print("should i show the immage? "+ str(show))
		if show: 
			cv2.imshow(pictureName,capture)
			cv2.waitKey(1000) & 0xFF
			cv2.destroyAllWindows()


def main():
	# ---- global variables with  DEFAULT VALUES ---
	marker = False
	show = False
	
	parser = argparse.ArgumentParser( description="Saves snapshot from the camera.")
	parser.add_argument("--show", default=False, type=bool, help="do you want to display each picture you take?")
	parser.add_argument("--marker", default=False, type=bool, help="did you take a picture of the marker? True/ False")
	
	args = parser.parse_args()
	marker = args.marker
	show = args.show

	print(str(marker))
	print(str(show))

	if not os.path.exists("marker"):
		print("got no directory 'marker', will make one for you now")
		try:
			os.makedirs("marker")
		except:
			print("can not create a directory like that")

	if not os.path.exists("no_marker"):
		print("did not find directory 'no_maker', will build if for you now")
		try:
			os.makedirs("no_marker")
		except:
			print("can not create a directory like that")
	snapPicture(marker, show)

if __name__ == "__main__":
	main()
#not main no more
