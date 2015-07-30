import numpy as np
import cv2
import time

#loading images into variables
img = cv2.imread('messi5.jpg', 0)
image = cv2.imread('cube.jpeg', 1)
print("dbg1")


cv2.imshow('image', img)
k = cv2.waitKey(0) & 0xFF  # test if key pressed is esc
if k == 27:
    print("exiting")
    cv2.destroyAllWindows()  # test if key pressed is c
elif k == ord('c'):
    cv2.destroyAllWindows()
    print("showing cube")
    cv2.imshow('cube', image)
    key = cv2.waitKey(0)  # test if key pressed is f
    if key == ord('f'):
        cv2.imwrite('output.jpg', image)
elif k == ord('s'):  # test if key pressed is s
    print("deleting windows")
    cv2.destroyAllWindows()


#use cv2.namedWindow() to create a window and load an image later
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
time.sleep(5)
cv2.imshow('image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()