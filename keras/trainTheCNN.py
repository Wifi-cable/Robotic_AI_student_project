# program to train Convolution Neural Network to detect toys and not toys


# import the necessary packages
import matplotlib.pyplot as plt
from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import Adam, SGD, Adadelta, Adagrad
from sklearn.model_selection import train_test_split
from keras.preprocessing.image import img_to_array
from keras.utils import to_categorical
from keras.utils.vis_utils import plot_model
from keras.models import Sequential
from keras.layers.convolutional import Conv2D
from keras.layers.convolutional import MaxPooling2D
from keras.layers.core import Activation
from keras.layers.core import Flatten
from keras.layers.core import Dense
from keras import backend as K
from imutils import paths
import numpy as np
import random
import cv2
import os
import time
import matplotlib.pyplot as plt


# data files
# choose image directory
IMAGE_DIRECTORY = "images/"
OUTPUT_DIRECTORY = "out"

# Set up the number of training passes (epochs), learning rate, and batch size
EPOCHS = 10
LEARNING_RATE = 1e-4
BATCH_SIZE = 32
IMG_WIDTH = 160
IMG_HEIGHT = 120

# set up output template for file saving later, also mkdir folder if necessary
OUTPUT_TEMPLATE = "{}{}x{}:{}@{}".format(OUTPUT_DIRECTORY+os.path.sep, IMG_WIDTH,
                                         IMG_HEIGHT, EPOCHS, BATCH_SIZE)
if not os.path.isdir(OUTPUT_DIRECTORY):
    os.mkdir(OUTPUT_DIRECTORY)


class LeNet:
    @staticmethod
    def build(width, height, depth, classes):
        # initialize the model
        model = Sequential()
        inputShape = (height, width, depth)

        # if we are using "channels first", update the input shape
        if K.image_data_format() == "channels_first":
            inputShape = (depth, height, width)

        # First layer is a convolution with 20 functions and a kernel size of 5x5 (2 neighbor pixels on each side)
        model.add(Conv2D(20, (5, 5), padding="same",
                         input_shape=inputShape))
        # our activation function is ReLU (Rectifier Linear Units)
        model.add(Activation("relu"))
        # second layer is maxpooling 2x2 that reduces our image resolution by half
        model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        # Third Layer - Convolution, twice the size of the first convoltion
        model.add(Conv2D(40, (5, 5), padding="same"))
        model.add(Activation("relu"))

        model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        # Fifth Layer is Full connected flattened layer that makes our 3D images into 1D arrays
        model.add(Flatten())
        model.add(Dense(500))
        model.add(Activation("relu"))

        # softmax classifier
        model.add(Dense(classes))
        model.add(Activation("softmax"))

        # save plot of model for documenting reasons
        plot_model(model, to_file=OUTPUT_TEMPLATE +
                   "_model.png", show_shapes=True)
        # return the completed network architecture
        return model


# initialize the data and labels
print("Loading training images set...")
data = []
labels = []

# grab the images from the directories and randomly shuffle them
imagePaths = sorted(list(paths.list_images(IMAGE_DIRECTORY)))
# use a random seed number from the time clock
seed = int(time.time() % 1000)
random.seed(seed)
random.shuffle(imagePaths)
print("number of images", len(imagePaths))
print(imagePaths[0])
m = 0
nm = 0
# loop over the input images
for imagePath in imagePaths:
    # load the image, pre-process it, and store it in the data list
    image = cv2.imread(imagePath)
    image = cv2.resize(image, (IMG_WIDTH, IMG_HEIGHT))
    image = img_to_array(image)
    data.append(image)

    # extract the class label from the image path and update the
    # labels list
    label = imagePath.split(os.path.sep)[-2]

    if label == "marker":
        label = 1
        m += 1
    else:
        label = 0
        nm += 1
    labels.append(label)

# Normalize the values of the pixels to be between 0 and 1 instead of 0 to 255
data = np.array(data, dtype="float") / 255.0
labels = np.array(labels)

print(labels)
print("#markers", m, "#no markers", nm)

# split the data between a testing set and a training set
(trainX, testX, trainY, testY) = train_test_split(data,
                                                  labels, test_size=0.20, random_state=seed)

# convert the labels from integers to vectors
trainY = to_categorical(trainY, num_classes=2)
testY = to_categorical(testY, num_classes=2)

# construct the image generator for data augmentation
aug = ImageDataGenerator(rotation_range=40, width_shift_range=0.2,
                         height_shift_range=0.2, shear_range=0.1, zoom_range=0.2,
                         horizontal_flip=True, fill_mode="nearest")

# initialize the weights of the network
print("compiling CNN network...")
cnNetwork = LeNet.build(width=IMG_WIDTH, height=IMG_HEIGHT, depth=3, classes=2)
# use adam optimizer.  tried SGD for comparison - needs more epochs (>100)
opt = Adam(lr=LEARNING_RATE, decay=LEARNING_RATE / EPOCHS)
#opt = SGD(lr=LEARNING_RATE,decay=LEARNING_RATE / EPOCHS)

# if we have more than two classes, use categorical crossentropy
cnNetwork.compile(loss="binary_crossentropy", optimizer=opt,
                  metrics=["accuracy"])

# train the network - here is where the magic happens...
print("training network...")
print("length trainx", len(trainX), " length trainy ", len(trainY))

history = cnNetwork.fit_generator(aug.flow(trainX, trainY, batch_size=BATCH_SIZE),
                                  validation_data=(testX, testY), steps_per_epoch=len(
    trainX) // BATCH_SIZE,
    epochs=EPOCHS, verbose=1)

print("Output directory is {}".format(OUTPUT_DIRECTORY))
print("Image dimensions are {}x{}".format(IMG_WIDTH, IMG_HEIGHT))
print("Trained using {} Epochs and {} batch size".format(EPOCHS, BATCH_SIZE))
print("Saving graphs and model to {}_accuracy/_loss/.model".format(OUTPUT_TEMPLATE))
# summarize history for accuracy
plt.plot(history.history['accuracy'])
plt.plot(history.history['val_accuracy'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.savefig(OUTPUT_TEMPLATE+"_accuracy")

# summarize history for loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.savefig(OUTPUT_TEMPLATE+"_loss")

# save the CNN network weights to file
cnNetwork.save(OUTPUT_TEMPLATE+".model")
