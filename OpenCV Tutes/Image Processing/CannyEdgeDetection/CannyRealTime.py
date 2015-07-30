# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt


def nothing(x):
	pass

img = cv2.imread('cube.jpg')
edges = img
cv2.namedWindow('Canny Real-Time')

cv2.createTrackbar('MinVal', 'Canny Real-Time', 0, 500, nothing)
cv2.createTrackbar('MaxVal', 'Canny Real-Time', 0, 500, nothing)

while(True):
	cv2.imshow('Canny Real-Time', edges)
	k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break
	minVal = cv2.getTrackbarPos('MinVal', 'Canny Real-Time')
	maxVal = cv2.getTrackbarPos('MaxVal', 'Canny Real-Time')
	edges = cv2.Canny(img, minVal, maxVal)

cv2.destroyAllWindows()