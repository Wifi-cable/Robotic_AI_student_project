#!/usr/bin/env python3
from cv2 import cv2  # opencv
import enum
from keras.models import load_model
from keras.preprocessing.image import img_to_array
import numpy  # for opencv
import os  # other fileIO stuff
import rospy  # need Ros Python
from std_msgs.msg import String  # ROS python needs a Ros message to publish
import sys  # fileIO stuff

from snap_data_img import snapPicture
'''
TODO
- take picture ✔
- use opencv to load picture in variable ✔
- cut picture in 9 pices ✔
- deploy model of Neural Net ❓
- seach for markler in pices (or in whole picture first 𝙭) ✔
- figure out if marker is in left, right, center or nix. ✔
- publish result as string ✔

'''

# change these as needed


class Move(enum.Enum):
    Left = "left"
    Right: "right"
    Forward: "forward"
    NotFound: "notfound"


IMG_WIDTH = 160
IMG_HEIGHT = 160

model = load_model('/../keras/213x160:800@32:0.model')

# instead of return you publish
pub = rospy.Publisher('Marker', String, queue_size=10)
# naming a node makes a lot of sense for debugging. choose any name.
rospy.init_node('find_marker')
myRate = rospy.Rate(10)  # 10hz	#10 may need adjustment

# not while true,  it is while not ctrl+C from terminal.
while not rospy.is_shutdown():
    capture = snapPicture(False, False, False)  # snap image, save to capture
    # ! unsure if this works, may need to use cv2.imread(capture)
    image = cv2.resize(capture, (IMG_WIDTH, IMG_HEIGHT))

    tiles = []
    results = []

    # split into 9 tiles
    step_width = IMG_WIDTH // 3
    step_height = IMG_HEIGHT // 3
    for i in range(3):
        for j in range(3):
            x_start, x_end = i * step_width, (i + 1) * step_width
            y_start, y_end = j * step_height, (j + 1) * step_height

            tiles.append(cv2.resize(
                image[x_start:x_end, y_start:y_end], (IMG_WIDTH, IMG_HEIGHT)))
            print("{}:{}x{}:{}".format(x_start, x_end, y_start, y_end))

    # predict every tile and append to result
    for tile in tiles:
        tile = numpy.expand_dims(img_to_array(
            tile.astype("float") / 255.0), axis=0)

        (nm, m) = model.predict(tile)[0]
        probability = max(nm, m) * 100
        has_marker = m > nm

        results.append([has_marker, probability])

    # sort results into columns & calculate average
    # super weird code but works haha
    columns = []
    for i in range(3):
        column = [0, 0.0]  # [number of markers in column, average probability]
        for j in range(3):
            if results[i + j * 3][0] == True:
                column[0] += 1
                column[1] += results[i + j * 3][1]
        if column[0] > 0:
            # calculate average probability
            column[1] = column[1] / column[0]
        columns.append(column)

    # publish where to go
    # max(columns) does sort by num of markers first, avg. probability second
    if columns[0] == max(columns) and columns[0] == [0, 0.0]:
        print("No marker detected anywhere.")
        pub.publish(Move.NotFound)
    elif columns[0] == max(columns):
        print("Go left, I'm {}% sure".format(columns[0][1]))
        pub.publish(Move.Left)
    elif columns[1] == max(columns):
        print("Go forward, I'm {}% sure".format(columns[1][1]))
        pub.publish(Move.Forward)
    elif columns[2] == max(columns):
        print("Go right, I'm {}% sure".format(columns[2][1]))
        pub.publish(Move.Right)
    else:
        print("Should not happen.")

    myRate.sleep()  # like delay. to not spam my terminal 100 times per second
