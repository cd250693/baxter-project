# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('cube.jpg', 0)

laplacian = cv2.Laplacian(img, cv2.CV_64F)
sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=5)
sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=5)

titles = ['Original', 'Laplacian', 'Sobel X', 'Sobel Y']
images = [img, laplacian, sobelx, sobely]

for i in range(4):
	plt.subplot(2, 2, i + 1), plt.imshow(images[i], cmap='gray')
	plt.title(titles[i]), plt.xticks([]), plt.yticks([])

plt.show()