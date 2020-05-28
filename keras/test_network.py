# Test Network Program
# part of the toy  detector
# uses LeNet to detect if toys are in the image
#
# Francis X. Govers 2018
#
# references:
# https://www.pyimagesearch.com/2017/12/18/keras-deep-learning-raspberry-pi/
#
# import the necessary packages
from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np
from imutils import paths
import cv2
import os
import sys

# result vars
tp = 0
tpa = 0
fp = 0
fpa = 0
tn = 0
tna = 0
fn = 0
fna = 0


# load the trained convolutional neural network
print("[INFO] loading network...")
model = load_model("marker_no_marker.model")

imagePaths = sorted(list(paths.list_images("images/")))
for imagePath in imagePaths:
    # load image, preprocess it
    image = cv2.imread(imagePath)
    image = cv2.resize(image, (160, 120))
    image = image.astype("float") / 255.0
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)

    # classify this image, print result
    label = imagePath.split(os.path.sep)[-2] == "marker"
    (nm, m) = model.predict(image)[0]
    probability = max(nm, m)
    has_marker = m > nm

    if label:
        if has_marker:
            tp += 1
            tpa += probability
            result = "true positive"
        else:
            fn += 1
            fna += probability
            result = "false negative"
    else:
        if has_marker:
            fp += 1
            fpa += probability
            result = "false positive"
        else:
            tn += 1
            tna += probability
            result = "true negative"

    print("{}: {} with {:.2f} probability".format(
        imagePath, result, probability))

print("##########RESULTS##########")
tpa = tpa / tp
fpa = fpa / fp
tna = tna / tn
fna = fna / fn
print("{} true positives,  {}% probability".format(tp, tpa))
print("{} false positives, {}% probability".format(fp, fpa))
print("{} true negatives,  {}% probability".format(tn, tna))
print("{} false negatives, {}% probability".format(fn, fna))

""" 

# load the image
image = cv2.imread(sys.argv[1])
orig = image.copy()

# pre-process the image for classification
image = cv2.resize(image, (320, 240))
image = image.astype("float") / 255.0
image = img_to_array(image)
image = np.expand_dims(image, axis=0)


# classify the input image
(nomarker, marker) = model.predict(image)[0]
print("Marker = ", marker, " No Marker = ", nomarker)
# build the label
label = "Marker" if marker > nomarker else "No Marker"
probability = max(marker, nomarker)
label = "{}: {:.2f}%".format(label, probability * 100)

# draw the label on the image
output = imutils.resize(orig, width=400)
cv2.putText(output, label, (200, 25),  cv2.FONT_HERSHEY_SIMPLEX,
			0.7, (200, 255, 40), 2)

# show the output image
cv2.imshow("Output", output)
cv2.imwrite("marker_classify2.jpg", output)
cv2.waitKey(0)
 """
