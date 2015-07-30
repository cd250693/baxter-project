import numpy as np
import cv2

cap = cv2.VideoCapture(0)
i = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the Resulting Frame
    cv2.imshow('gray', gray)
    cv2.imshow('color', frame)
    if i == 0:
        cv2.moveWindow('color', 710, 0)
        cv2.moveWindow('gray', 66, 0)
        i += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#When everything done, release the capture
cap.release()
cv2.destroyAllWindows()