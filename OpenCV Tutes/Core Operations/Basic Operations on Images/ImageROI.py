# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

#show img using matplotlib
img2 = cv2.imread('cube.jpg', 1)
img2 = img2[:, :, ::-1]
plt.imshow(img2, cmap='gray', interpolation='bicubic')
plt.xticks([]), plt.yticks([])
plt.show()

# take one region, copy it to another part of the image
img = cv2.imread('cube.jpg')
square = img[333:494, 163:328]
img[170:329, 163:328] = square
cv2.imshow('img', img)
cv2.waitKey(0)