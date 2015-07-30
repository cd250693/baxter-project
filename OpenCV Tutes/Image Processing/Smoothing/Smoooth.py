# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('cube.jpg')[:, :, ::-1]

kernel = np.ones((5, 5), np.float32) / 25
average = cv2.filter2D(img, -1, kernel)
blur = cv2.blur(img, (5, 5))
Gaussian = cv2.GaussianBlur(img, (5, 5), 0)
median = cv2.medianBlur(img, 5)
bilateral = cv2.bilateralFilter(img, 9, 75, 75)

titles = ['Original', 'Averaging', 'Blurred', 'Gaussian Blur', 'Median', 'Bilateral']
images = [img, average, blur, Gaussian, median, bilateral]

for i in range(len(images)):
	plt.subplot(2, 3, i + 1), plt.imshow(images[i]), plt.title(titles[i])
	plt.xticks([]), plt.yticks([])

plt.show()