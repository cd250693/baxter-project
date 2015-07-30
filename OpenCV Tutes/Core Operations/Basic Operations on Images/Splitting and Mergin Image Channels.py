# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('cube.jpg')
img = img[:, :, ::-1]

# split BGR into single planes
# NOTE # split is a costly operation, try use numpy indexing
b, g, r = cv2.split(img)

# merge back into one image
img2 = cv2.merge((b, g, r))

##### OR #####
b = img[:, :, 0]


# make red pixels zero. no need to split
img3 = cv2.imread('cube.jpg')
img3[:, :, 0] = 0

# blue pixels zero
img4 = cv2.imread('cube.jpg')
img4[:, :, 2] = 0

# green pixels zero
img5 = cv2.imread('cube.jpg')
img5[:, :, 1] = 0

plt.subplot(231), plt.imshow(img, cmap='gray'), plt.title('Original'),
plt.xticks([]), plt.yticks([])

plt.subplot(232), plt.imshow(img2, cmap='gray'), plt.title('Merged'),
plt.xticks([]), plt.yticks([])

plt.subplot(233), plt.imshow(img3, cmap='gray'), plt.title('Red zero')
plt.xticks([]), plt.yticks([])

plt.subplot(234), plt.imshow(img4), plt.title('Blue zero')
plt.xticks([]), plt.yticks([])

plt.subplot(235), plt.imshow(img5), plt.title('Green zero')
plt.xticks([]), plt.yticks([])

plt.show()