# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt

## SCALING

img = cv2.imread('messi5.jpg')

res = cv2.resize(img,None,fx=2, fy=2, interpolation = cv2.INTER_CUBIC)

cv2.imshow('scale', res)
cv2.waitKey(0)
cv2.destroyAllWindows()

#OR

height, width = img.shape[:2]
res2 = cv2.resize(img, (2 * width, 2 * height), interpolation=cv2.INTER_CUBIC)

cv2.imshow('scale alt', res2)
cv2.waitKey(0)
cv2.destroyAllWindows()

## TRANSLATION

img2 = cv2.imread('messi5.jpg', 0)
rows2, cols2 = img2.shape

M = np.float32([[1, 0, 100], [0, 1, 50]])
dst = cv2.warpAffine(img2, M, (cols2, rows2))

cv2.imshow('translate', dst)
cv2.waitKey(0)
cv2.destroyAllWindows()

## ROTATION

img3 = cv2.imread('messi5.jpg', 0)
rows3, cols3 = img3.shape

M2 = cv2.getRotationMatrix2D((cols3 / 2, rows3 / 2), 90, 1)
dst2 = cv2.warpAffine(img, M2, (cols3, rows3))

cv2.imshow('rotate 90', dst2)
cv2.waitKey(0)
cv2.destroyAllWindows()

## AFFINE TRANSFORM

feed = cv2.imread('messi5.jpg')
img4 = feed[:, :, ::-1]
rows4, cols4, ch4 = img4.shape

pts1 = np.float32([[50, 50], [200, 50], [50, 200]])
pts2 = np.float32([[10, 100], [200, 50], [100, 250]])

M4 = cv2.getAffineTransform(pts1, pts2)

dst4 = cv2.warpAffine(img4, M4, (cols4, rows4))

plt.subplot(121), plt.imshow(img4), plt.title('Input')
plt.subplot(122), plt.imshow(dst4), plt.title('Output')
plt.show()

## PERSPECTIVE TRANSFORM

img5 = cv2.imread('cube.jpg')[:, :, ::-1]
rows5, cols5, ch = img5.shape

pts15 = np.float32([[88, 107], [88, 224], [173, 91], [168, 196]])
pts25 = np.float32([[0, 0], [0, 200], [200, 0], [200, 200]])

M5 = cv2.getPerspectiveTransform(pts15, pts25)

dst5 = cv2.warpPerspective(img5, M5, (200, 200))

plt.subplot(121), plt.imshow(img5), plt.title('Input')
plt.subplot(122), plt.imshow(dst5), plt.title('Output')
plt.show()