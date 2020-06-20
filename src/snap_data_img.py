#!/usr/bin/env python3
import cv2
import datetime
import sys
import argparse
import os
import numpy


def undistort(imgName):
    img = imgName

    # need mtx and dist
    mtx = numpy.array([[1.520925808570831805e+03, 0.000000000000000000e+00, 7.713748717157859573e+02],
                       [0.000000000000000000e+00, 1.519652028136312083e+03,
                           5.851593992111154421e+02],
                       [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])
    dist = numpy.array([1.685848346581539092e-01, -5.558901276684754533e-01, -
                        4.224284338330285687e-03, -2.539218024257525446e-03, 5.383658295121591664e-01])
    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        mtx, dist, (w, h), 1, (w, h))

    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    x, y, w, h = roi  # cropping
    dst = dst[y:y+h, x:x+w]
    # cv2.imwrite('calibresult.png',dst)
    return dst


def snapPicture(marker=False, show=False, saveToFile=True):
    print("paramcheck marker=" + str(marker) + " show = "+str(show))
    dayTime = datetime.datetime.now()  # get unix time of when picture was taken
    now = dayTime.strftime("%s")  # save as string
    cap = cv2.VideoCapture(0)
    ret, capture = cap.read()
    pictureName = "data" + now + ".png"
    print(pictureName)
# if immage captured correctly save it.
    print("taking a picture was sucessfull? " + str(ret))
    # undistort
    #capture = undistort(capture)
    if saveToFile:
        if ret:
            if marker:  # save in folder with marker
                folder = "marker/"
                cv2.imwrite((folder + pictureName), capture)
            else:  # save in other folder
                folder = "no_marker/"
                cv2.imwrite((folder + pictureName), capture)
            print("saved a picture named " +
                  pictureName + " to the folder " + folder)

            #print("should i show the immage? "+ str(show))
            if show:
                cv2.imshow(pictureName, capture)
                cv2.waitKey(1000) & 0xFF
                cv2.destroyAllWindows()
    else:
        return capture


def main():
    # ---- global variables with  DEFAULT VALUES ---
    marker = False
    show = False

    parser = argparse.ArgumentParser(
        description="Saves snapshot from the camera.")
    parser.add_argument("--show", default=False, type=bool,
                        help="do you want to display each picture you take?")
    parser.add_argument("--marker", default=False, type=bool,
                        help="did you take a picture of the marker? True/ False")

    args = parser.parse_args()
    marker = args.marker
    show = args.show

    print(str(marker))
    # print(str(show))

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
    snapPicture(marker, show, True)


if __name__ == "__main__":
    main()
# not main no more
