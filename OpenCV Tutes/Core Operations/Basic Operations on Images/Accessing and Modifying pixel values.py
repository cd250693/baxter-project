# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

# load image
img = cv2.imread('cube.jpg')
# show img
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
#show img using matplotlib
img2 = cv2.imread('cube.jpg', 1)
img2 = img2[:, :, ::-1]
plt.imshow(img2, cmap='gray', interpolation='bicubic')
plt.xticks([]), plt.yticks([])
plt.show()


# access pixel value by its row and column coordinates
px = img[250, 250]
print(px)

#accessing only blue pixel, prints out number of them
blue = img[254, 0, 0]
print(blue)

# modify the pixel the same way
img[100, 100] = [255, 255, 255]
print(img[100, 100])

#better pixel accessing and editing method
#print RED value
print(img.item(10, 10, 2))

# modify RED value
img.itemset((10, 10, 2), 100)
print(img.item(10, 10, 2))
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()