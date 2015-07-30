# -*- coding: utf-8 -*-
import cv2
import numpy as np

img = cv2.imread('cube.jpg')

# returns tuple of number of row, columns and channels(if colour)
# if image is grayscale, only rows and columns will be returned
# so its a good way to check if the img is colour or grayscale
print(img.shape)

# access total number of pixels
print(img.size)

# access img data type
print(img.dtype)
# very important, as many errors are caused by invalid datatype