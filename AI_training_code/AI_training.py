'''SOURCE : https://gitlab.com/f2m2robserv/lenet#lenet-5-in-9-lines-of-code-using-keras '''

from requests import get

def download_file(url, file_name):
    with open(file_name, "wb") as file:
        response = get(url)
        file.write(response.content)

download_file('https://raw.githubusercontent.com/hamlinzheng/mnist/refs/heads/master/dataset/train-images-idx3-ubyte.gz', 'train-images-idx3-ubyte.gz')
download_file('https://raw.githubusercontent.com/hamlinzheng/mnist/refs/heads/master/dataset/train-labels-idx1-ubyte.gz', 'train-labels-idx1-ubyte.gz')
download_file('https://raw.githubusercontent.com/hamlinzheng/mnist/refs/heads/master/dataset/t10k-images-idx3-ubyte.gz', 't10k-images-idx3-ubyte.gz')
download_file('https://raw.githubusercontent.com/hamlinzheng/mnist/refs/heads/master/dataset/t10k-labels-idx1-ubyte.gz', 't10k-labels-idx1-ubyte.gz')

import gzip
import numpy as np
import pandas as pd
from time import time

from sklearn.model_selection import train_test_split
import tensorflow as tf
import keras
import keras.layers as layers
from keras.models import Sequential
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from keras.utils import to_categorical
from keras.callbacks import TensorBoard

%matplotlib inline 
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()

# If errors happen below, they might not be blocking, and may depend of your GPU hardware

def read_mnist(images_path: str, labels_path: str):
    with gzip.open(labels_path, 'rb') as labelsFile:
        labels = np.frombuffer(labelsFile.read(), dtype=np.uint8, offset=8)

    with gzip.open(images_path,'rb') as imagesFile:
        length = len(labels)
        # Load flat 28x28 px images (784 px), and convert them to 28x28 px
        features = np.frombuffer(imagesFile.read(), dtype=np.uint8, offset=16) \
                        .reshape(length, 784) \
                        .reshape(length, 28, 28, 1)

    return features, labels

train = {}
test = {}

train['features'], train['labels'] = read_mnist('train-images-idx3-ubyte.gz', 'train-labels-idx1-ubyte.gz')
test['features'], test['labels'] = read_mnist('t10k-images-idx3-ubyte.gz', 't10k-labels-idx1-ubyte.gz')

print('# of training images:', train['features'].shape[0])
print('# of test images:', test['features'].shape[0])

def display_image(position):
    image = train['features'][position].squeeze()
    plt.title('Example %d. Label: %d' % (position, train['labels'][position]))
    plt.imshow(image, cmap=plt.cm.gray_r)

display_image(0)
display_image(1)
display_image(2)

train_labels_count = np.unique(train['labels'], return_counts=True)
dataframe_train_labels = pd.DataFrame({'Label':train_labels_count[0], 'Count':train_labels_count[1]})
dataframe_train_labels

validation = {}
train['features'], validation['features'], train['labels'], validation['labels'] = train_test_split(train['features'], train['labels'], test_size=0.2, random_state=0)

print('# of training images:', train['features'].shape[0])
print('# of validation images:', validation['features'].shape[0])

# Pad images with 0s
train['features']      = np.pad(train['features'], ((0,0),(2,2),(2,2),(0,0)), 'constant')
validation['features'] = np.pad(validation['features'], ((0,0),(2,2),(2,2),(0,0)), 'constant')
test['features']       = np.pad(test['features'], ((0,0),(2,2),(2,2),(0,0)), 'constant')

print("Updated Image Shape: {}".format(train['features'][0].shape))

model = keras.Sequential()

model.add(layers.Conv2D(filters=6, kernel_size=(3, 3), activation='relu', input_shape=(32,32,1)))

model.add(layers.AveragePooling2D(pool_size=(2, 2)))

model.add(layers.Conv2D(filters=16, kernel_size=(3, 3), activation='relu'))

model.add(layers.AveragePooling2D(pool_size=(2, 2)))

model.add(layers.Flatten())

model.add(layers.Dense(units=120, activation='relu'))

model.add(layers.Dense(units=84, activation='relu'))

model.add(layers.Dense(units=10, activation = 'softmax'))

# Here we can see the summary of all layers of the model
model.summary()

model.compile(loss=keras.losses.categorical_crossentropy, optimizer=keras.optimizers.Adam(), metrics=['accuracy'])

EPOCHS = 10
BATCH_SIZE = 128

X_train, y_train = train['features'], to_categorical(train['labels'])
X_validation, y_validation = validation['features'], to_categorical(validation['labels'])

train_generator = ImageDataGenerator().flow(X_train, y_train, batch_size=BATCH_SIZE)
validation_generator = ImageDataGenerator().flow(X_validation, y_validation, batch_size=BATCH_SIZE)

print('# of training images:', train['features'].shape[0])
print('# of validation images:', validation['features'].shape[0])

steps_per_epoch = X_train.shape[0] // BATCH_SIZE
validation_steps = X_validation.shape[0] // BATCH_SIZE

tensorboard = TensorBoard(log_dir="logs/{}".format(time()))

model.fit(train_generator,
          steps_per_epoch=steps_per_epoch,
          epochs=EPOCHS,
          validation_data=validation_generator,
          validation_steps=validation_steps,
          shuffle=True,
          callbacks=[tensorboard])

score = model.evaluate(test['features'], to_categorical(test['labels']))
print('Test loss:', score[0])
print('Test accuracy:', score[1])

from tensorflow.keras.preprocessing.image import load_img, img_to_array
from tensorflow.keras.models import load_model
from PIL import Image

# Load the image to be classified
image_path = 'number5.png'
image = load_img(image_path, color_mode='grayscale', target_size=(32, 32))

# Invert black/white colors
image = Image.fromarray((255 - img_to_array(image)).squeeze())

# Display the image
plt.imshow(image, cmap='gray')
plt.title('Loaded Image')
plt.axis('off')
plt.show()

# Preprocess the image for the model
image_array = img_to_array(image)
image_array = np.expand_dims(image_array, axis=0)

# Make an inference
inference = model.predict(image_array)
predicted_class = np.argmax(inference, axis=1)
print('Inferred class:', predicted_class)