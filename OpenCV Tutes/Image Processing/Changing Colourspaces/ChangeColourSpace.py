# -*- coding: utf-8 -*-
import cv2

# get all flags in cv2 that begin with COLOR_
flags = [i for i in dir(cv2) if i.startswith('COLOR_')]
print(flags)

# we commonly use cv2.COLOR_BGR2GRAY and cv2.COLOR_BGR2HSV