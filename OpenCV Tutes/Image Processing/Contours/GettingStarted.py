# -*- coding: utf-8 -*-
import cv2
import numpy as np

img = cv2.imread('rectangle.jpg', 0)

ret, thresh = cv2.threshold(img, 127, 255, 0)
ret, thresh1 = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


cv2.imshow('original', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imshow('thresh', thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imshow('thresh1', thresh1)
cv2.waitKey(0)
cv2.destroyAllWindows()

j = -1
img1 = img.copy()
img2 = img.copy()
img3 = img.copy()
imgs = [img1, img2, img3]
for i in range(3):
	cv2.drawContours(imgs[i], contours, j, (100, 255, 0), 3)
	cv2.imshow('contour', imgs[i])
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	j += 1

#cv2.drawContours(img, contours, 0, (100, 255, 0), 3)

#cv2.imshow('contour 0', img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

#cv2.drawContours(img, contours, 1, (100, 255, 0), 3)

#cv2.imshow('contour 1', img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
