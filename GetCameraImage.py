# -*- coding: utf-8 -*-
import cv2


def GetCamImage():
    cap = cv2.VideoCapture(0)
    i = 0
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # Display the Resulting Frame
        cv2.imshow('Camera Feed', frame)
        if i == 0:
            cv2.moveWindow('Camera Feed', 70, 0)
            i += 1
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print('Closing Camera Feed')
            return False
        if key == ord('c'):
            cv2.imwrite('1-output.jpg', frame)
            print('Saving Image')
            return True
    #When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

GetCamImage()