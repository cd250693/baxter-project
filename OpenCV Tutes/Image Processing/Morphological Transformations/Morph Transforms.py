# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('j.png')
imgopening = cv2.imread('opening.png')
imgclosing = cv2.imread('closing.png')
kernel = np.ones((5, 5), np.uint8)

original = img

erosion = cv2.erode(img, kernel, iterations=1)

dilation = cv2.dilate(img, kernel, iterations=1)

gradient = cv2.morphologyEx(img, cv2.MORPH_GRADIENT, kernel)

openoriginal = imgopening

opening = cv2.morphologyEx(imgopening, cv2.MORPH_OPEN, kernel)

closingoriginal = imgclosing

closing = cv2.morphologyEx(imgclosing, cv2.MORPH_CLOSE, kernel)

tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)

blackhat = cv2.morphologyEx(img, cv2.MORPH_BLACKHAT, kernel)

# display all images in a single window

titles = ['Original', 'Erosion', 'Dilation', 'Gradient',
	'Open Original', 'Opening', 'Closing Original', 'Closing',
	'Tophat', 'Blackhat']
images = [original, erosion, dilation, gradient,
	openoriginal, opening, closingoriginal, closing,
	tophat, blackhat]
for i in range(10):
	plt.subplot(3, 4, i + 1), plt.imshow(images[i])
	plt.title(titles[i])
	plt.xticks([]), plt.yticks([])
plt.show()