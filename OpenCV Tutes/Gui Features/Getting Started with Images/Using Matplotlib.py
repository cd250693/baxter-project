import numpy as np
import cv2
from matplotlib import pyplot as plt

print('dbg')
img = cv2.imread('cube.jpeg', 1)
plt.imshow(img, cmap='gray', interpolation='bicubic')
plt.xticks([]), plt.yticks([])  # to hide tick values on x and y axis
plt.show()


# open file, while correcting colours.
# OpenCV uses BGR and Matplotlib uses RGB.
# We must reorganise the colour info
img2 = img[:, :, ::-1]  # this changes the BGR to RGB
plt.imshow(img2)
plt.xticks([]), plt.yticks([])
plt.show()