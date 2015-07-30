# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time

img1 = cv2.imread('messi5.jpg')

e1 = cv2.getTickCount()
# your code execution
# ---example code---
for i in xrange(5, 49, 2):
    img1 = cv2.medianBlur(img1, i)

# ^^^^^^^^^^^^^^^^^^^^^
e2 = cv2.getTickCount()
t = (e2 - e1) / cv2.getTickFrequency()
print t

##----------OR----------##


t1 = time.time()
# Code bellow
for i in xrange(5, 49, 2):
    img2 = cv2.medianBlur(img1, i)
#^^^^^^^^^^^^^^^^^^^^
t2 = time.time()
passed = t2 - t1
print passed