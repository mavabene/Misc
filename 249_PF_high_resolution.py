# -*- coding: utf-8 -*-
"""High Resolution.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1h0Z1seoi37Eyd9JIfNTfAqTPtc0wvu2k
"""

import keras
keras.__version__

import os, shutil
original_dir=  #place here direction to directory with images

# The directory where we will
# store our smaller dataset
base_dir = #create a folder named "conv" in the directory
os.mkdir(base_dir)

# Directories for our training,
# validation and test splits
train_dir = os.path.join(base_dir, 'train')
os.mkdir(train_dir)
validation_dir = os.path.join(base_dir, 'validation')
os.mkdir(validation_dir)

#Create:
# A directory with our training pictures of cut marks on defleshed bones
train_nm_dir = os.path.join(train_dir, 'NM')
os.mkdir(train_nm_dir)

# Directory with our training pictures of cut marks on fleshed bones
train_wm_dir = os.path.join(train_dir, 'WM')
os.mkdir(train_wm_dir)

# Directory with our validation pictures of cut marks on defleshed bones
validation_nm_dir = os.path.join(validation_dir, 'NM')
os.mkdir(validation_nm_dir)

# Directory with our validation pictures of cut marks on fleshed bones
validation_wm_dir = os.path.join(validation_dir, 'WM')
os.mkdir(validation_wm_dir)



# Copy first CM images to train_nm_dir
fnames = ['NM.{}.bmp'.format(i) for i in range(13)]
for fname in fnames:
    src = os.path.join(original_dir, fname)
    dst = os.path.join(train_nm_dir, fname)
    shutil.copyfile(src, dst)

# Copy next CM images to validation_nm_dir
fnames = ['NM.{}.bmp'.format(i) for i in range(13, 19)]
for fname in fnames:
    src = os.path.join(original_dir, fname)
    dst = os.path.join(validation_nm_dir, fname)
    shutil.copyfile(src, dst)
    

    
# Copy first CM images to train_wm_dir
fnames = ['WM.{}.bmp'.format(i) for i in range(19)]
for fname in fnames:
    src = os.path.join(original_dir, fname)
    dst = os.path.join(train_wm_dir, fname)
    shutil.copyfile(src, dst)
    
# Copy next 10 CM images to validation_wm_dir
fnames = ['WM.{}.bmp'.format(i) for i in range(19, 27)]
for fname in fnames:
    src = os.path.join(original_dir, fname)
    dst = os.path.join(validation_wm_dir, fname)
    shutil.copyfile(src, dst)

from keras import layers
from keras import models

#height, width, channels
model = models.Sequential()
model.add(layers.Conv2D(32, (3, 3), activation='relu',
                        input_shape=(80, 400, 3)))#cambiar 1 a 3 si es color
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(128, (3, 3), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(128, (3, 3), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Flatten())
model.add(layers.Dense(512, activation='relu'))
model.add(layers.Dense(1, activation='sigmoid'))

#ALEXNET
import tensorflow
import keras
from keras.models import Sequential
from keras.layers import Dense, Activation, Dropout, Flatten, Conv2D, MaxPooling2D
from keras.layers.normalization import BatchNormalization
import numpy as np
np.random.seed(1000)

# (3) Create a sequential model
model = Sequential()

# 1st Convolutional Layer
model.add(Conv2D(filters=96, input_shape=(80,400,3), kernel_size=(9,9),\
 strides=(4,4), padding='valid'))
model.add(Activation('relu'))
# Pooling 
model.add(MaxPooling2D(pool_size=(2,2), strides=(2,2), padding='valid'))
# Batch Normalisation before passing it to the next layer
model.add(BatchNormalization())

# 2nd Convolutional Layer
model.add(Conv2D(filters=256, kernel_size=(9,9), strides=(1,1), padding='valid'))
model.add(Activation('relu'))
# Pooling
model.add(MaxPooling2D(pool_size=(1,1), strides=(2,2), padding='valid'))
# Batch Normalisation
model.add(BatchNormalization())

# 3rd Convolutional Layer
model.add(Conv2D(filters=384, kernel_size=(1,1), strides=(1,1), padding='valid'))
model.add(Activation('relu'))
# Batch Normalisation
model.add(BatchNormalization())

# 4th Convolutional Layer
model.add(Conv2D(filters=384, kernel_size=(1,1), strides=(1,1), padding='valid'))
model.add(Activation('relu'))
# Batch Normalisation
model.add(BatchNormalization())

# 5th Convolutional Layer
model.add(Conv2D(filters=256, kernel_size=(1,1), strides=(1,1), padding='valid'))
model.add(Activation('relu'))
# Pooling
model.add(MaxPooling2D(pool_size=(1,1), strides=(2,2), padding='valid'))
# Batch Normalisation
model.add(BatchNormalization())

# Passing it to a dense layer
model.add(Flatten())
# 1st Dense Layer
model.add(Dense(4096, input_shape=(80*400*3,)))
model.add(Activation('relu'))
# Add Dropout to prevent overfitting
model.add(Dropout(0.4))
# Batch Normalisation
model.add(BatchNormalization())

# 2nd Dense Layer
model.add(Dense(4096))
model.add(Activation('relu'))
# Add Dropout
model.add(Dropout(0.4))
# Batch Normalisation
model.add(BatchNormalization())

# 3rd Dense Layer
model.add(Dense(1000))
model.add(Activation('relu'))
# Add Dropout
model.add(Dropout(0.4))
# Batch Normalisation
model.add(BatchNormalization())

# Output Layer
model.add(Dense(3))
model.add(Activation('sigmoid'))

from keras import optimizers

model.compile(loss='binary_crossentropy',
              optimizer=optimizers.RMSprop(lr=1e-4),
              metrics=['acc'])

from keras.preprocessing.image import ImageDataGenerator

# All images will be rescaled by 1./255
train_datagen = ImageDataGenerator(rescale=1./255)
test_datagen = ImageDataGenerator(rescale=1./255)

train_generator = train_datagen.flow_from_directory(
        # This is the target directory
        train_dir,
        # All images will be resized to 40x160
        target_size=(80, 400),
        batch_size=20,
        # Since we use binary_crossentropy loss, we need binary labels
        class_mode='binary')

validation_generator = test_datagen.flow_from_directory(
        validation_dir,
        target_size=(80, 400),
        batch_size=20,
        class_mode='binary')

history = model.fit_generator(
      train_generator,
      steps_per_epoch=100,
      epochs=100,
      validation_data=validation_generator,
      validation_steps=50)

#Model to use with image augmentation
from keras.preprocessing.image import ImageDataGenerator
#Let's train our network using data augmentation and dropout:
train_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=40,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,)

# Note that the validation data should not be augmented!
test_datagen = ImageDataGenerator(rescale=1./255)

train_generator = train_datagen.flow_from_directory(
        # This is the target directory
        train_dir,
        # All images will be resized to 150x150
        target_size=(80, 400),
        batch_size=20,
        # Since we use binary_crossentropy loss, we need binary labels
        class_mode='categorical')#put "categorical" si son más de 2 categorías

validation_generator = test_datagen.flow_from_directory(
        validation_dir,
        target_size=(80, 400),
        batch_size=20,
        class_mode='categorical')# use "categorical" para mas de dos grupos

history = model.fit_generator(
      train_generator,
      steps_per_epoch=3,#modify steps accordingly
      epochs=100,
      validation_data=validation_generator,
      validation_steps=3)#modify steps accordingly