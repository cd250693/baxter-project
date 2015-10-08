# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
###############################################################################
# Tests the image from the camera with varying ammout of time to adjust
###############################################################################


def GetCamImage(x):
    cap = cv2.VideoCapture(0)
    i = 0
    while i < x:
        ret, frame = cap.read()
        time.sleep(0.001)
        i += 1
    facemat = frame[:, :, ::-1]
    #When everything done, release the capture
    cap.release()
    return facemat


# get all of the faces, each one needs to call GetCamImage
one = GetCamImage(1)
print('Done 1')
two = GetCamImage(2)
print('Done 2')
three = GetCamImage(3)
print('Done 3')
four = GetCamImage(4)
print('Done 4')
five = GetCamImage(5)
print('Done 5')
six = GetCamImage(6)
print('Done 6')
seven = GetCamImage(7)
print('Done 7')
eight = GetCamImage(8)
print('Done 8')
nine = GetCamImage(9)
print('Done 9')
ten = GetCamImage(10)
print('Done 10')
# setup a matplot with all the faces (arranged like the cube was folded out)
plt.subplot(4, 3, 1), plt.imshow(one), plt.title('1'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 2), plt.imshow(two), plt.title('2'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 3), plt.imshow(three), plt.title('3'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 4), plt.imshow(four), plt.title('4'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 5), plt.imshow(five), plt.title('5'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 6), plt.imshow(six), plt.title('6'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 7), plt.imshow(seven), plt.title('7'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 8), plt.imshow(eight), plt.title('8'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 9), plt.imshow(nine), plt.title('9'),
plt.xticks([]), plt.yticks([])
plt.subplot(4, 3, 10), plt.imshow(ten), plt.title('10'),
plt.xticks([]), plt.yticks([])
plt.show()
