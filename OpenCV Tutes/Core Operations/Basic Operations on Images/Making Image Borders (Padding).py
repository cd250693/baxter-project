# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

BLUE = [0, 0, 255]

img = cv2.imread('cube.jpg')
img1 = img[:, :, ::-1]

replicate = cv2.copyMakeBorder(img1, 100, 100, 100, 100, cv2.BORDER_REPLICATE)
reflect = cv2.copyMakeBorder(img1, 100, 100, 100, 100, cv2.BORDER_REFLECT)
reflect101 = cv2.copyMakeBorder(img1, 100, 100, 100,
100, cv2.BORDER_REFLECT_101)
wrap = cv2.copyMakeBorder(img1, 100, 100, 100, 100, cv2.BORDER_WRAP)
constant = cv2.copyMakeBorder(img1, 100, 100, 100, 100,
    cv2.BORDER_CONSTANT, value = BLUE)

plt.subplot(231), plt.imshow(img1, 'gray'), plt.title('ORIGINAL'),
plt.xticks([]), plt.yticks([])
plt.subplot(232), plt.imshow(replicate, 'gray'), plt.title('REPLICATE'),
plt.xticks([]), plt.yticks([])
plt.subplot(233), plt.imshow(reflect, 'gray'), plt.title('REFLECT'),
plt.xticks([]), plt.yticks([])
plt.subplot(234), plt.imshow(reflect101, 'gray'), plt.title('REFLECT_101'),
plt.xticks([]), plt.yticks([])
plt.subplot(235), plt.imshow(wrap, 'gray'), plt.title('WRAP'),
plt.xticks([]), plt.yticks([])
plt.subplot(236), plt.imshow(constant, 'gray'), plt.title('CONSTANT'),
plt.xticks([]), plt.yticks([])

plt.show()