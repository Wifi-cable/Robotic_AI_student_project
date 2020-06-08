from cv2 import cv2
from keras.models import load_model
from keras.preprocessing.image import img_to_array
import numpy as np
import sys

try:
    image = sys.argv[1]
except:
    print("Usage: python find_marker.py path_to_image")
    sys.exit(-1)

NUM_TILES = 3
IMG_WIDTH = 160
IMG_HEIGHT = 120

model = load_model("marker_no_marker.model")
image = cv2.resize(cv2.imread(image), (IMG_WIDTH, IMG_HEIGHT))

# split into NUM_TILES tiles
tiles = []
results = []
step_width = IMG_WIDTH // (NUM_TILES + 1)
step_height = IMG_HEIGHT // (NUM_TILES + 1)
for i in range(NUM_TILES):
    for j in range(NUM_TILES):
        x_start = i * step_width
        x_end = (i+2) * step_width
        y_start = j * step_height
        y_end = (j+2) * step_height
        tiles.append(cv2.resize(
            image[x_start:x_end, y_start:y_end], (IMG_WIDTH, IMG_HEIGHT)))

        print("{}:{}x{}:{}".format(x_start, x_end, y_start, y_end))

# predict for every tile and append to result
num = 1
for tile in tiles:
    show = cv2.resize(tile.copy(), (640, 360))
    tile = np.expand_dims(img_to_array(tile.astype("float") / 255.0), axis=0)

    (nm, m) = model.predict(tile)[0]
    probability = max(nm, m) * 100
    has_marker = m > nm

    cv2.putText(show, "{}, {}% sure".format(has_marker, probability), (200, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 40), 2)
    cv2.imshow("Tile #{}".format(num), show)
    cv2.waitKey(2500)
    results.append([has_marker, probability])
    num += 1

# predict whole image for debugging purposes
(nm, m) = model.predict(tile)[0]
probability = max(nm, m) * 100
has_marker = m > nm

image = cv2.resize(image, (640, 480))
cv2.putText(image, "{}, {}% sure".format(has_marker, probability), (200, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 40), 2)
cv2.imshow("Image", image)
cv2.waitKey(2500)

print(results)
# sort results into columns
columns = []
for i in range(NUM_TILES):
    column = [0, 0.0]
    for j in range(NUM_TILES):
        if results[i + j * 3][0] == True:
            column[0] += 1
            column[1] += results[i + j * 3][1]
    if column[0] > 0:
        # calculate average probability
        column[1] = column[1] / column[0]
    columns.append(column)

print(columns)

if columns[0] == max(columns) and columns[0] == [0, 0.0]:
    print("You messed up and lost your marker. Well fucking done")
elif columns[0] == max(columns):
    print("should go left")
elif columns[1] == max(columns):
    print("should go forward")
elif columns[2] == max(columns):
    print("should go right")
else:
    print()
