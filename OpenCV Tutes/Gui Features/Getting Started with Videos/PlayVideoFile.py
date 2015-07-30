import numpy as np
import cv2

cap = cv2.VideoCapture('test.avi')
while(cap.isOpened()):
#Define frame as the video to display
    ret, frame = cap.read()
# used to convert the video before displaying
# Ensure video is expected colour
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#displaying the video frame in a window
# NOTE we are skipping the conversion
    cv2.imshow('frame', gray)
# by varying waitKey time, we can cause it to play faster/slower
# higher the number, slower it plays; lower the number, faster it plays
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print('done')