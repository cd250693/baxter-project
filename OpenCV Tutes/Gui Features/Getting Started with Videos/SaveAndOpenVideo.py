import numpy as np
import cv2
from time import sleep

cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('cube3.avi', fourcc, 20.0, (640, 480))
record = 0
print('to record, press r')
print('to exit, press q')

while(True):
    # ret and vid variables store the video feed
    ret, vid = cap.read()
    # show the variable vid in a window
    cv2.imshow('Video Feed', vid)
    # initialize the record variable
    # define keypressed variable, also wait for 1ms
    keypressed = cv2.waitKey(1) & 0xFF
    # set record variable to 1 when r is pressed
    if keypressed == ord('r'):
        record = 1
        print('now recording')
        print('to exit, press t')
    # test if t is pressed, and end recording if need be
    if keypressed == ord('t'):
        record = 0
        print('recording end')
    # writes vid to the ouput file when record is 1
    if record == 1:
        out.write(vid)
        sleep(0.025)
    # satement to break while loop when q is pressed
    if keypressed == ord('q'):
        print('exiting...')
        break

cv2.destroyWindow('Video Feed')
cap.release()

#print('view video? y/n')
#view = cv2.waitKey(0) & 0xFF
#if view == ord('y'):
rec = cv2.VideoCapture('cube3.avi')
print('now playing')
print('to exit, press q')
while(True):
    cv2.imshow('playback', rec)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        print('exiting...')
        break
rec.release()
cv2.destroyAllWindows()