# -*- coding: utf-8 -*-
# Gets the image from the connected camera
# NOTE: Camera needs to be mounted as a webcam
def GetCamImage():
    cap = cv2.VideoCapture(0)
    # i allows the video feed window to be set in place when created,
    #     but them moved afterwards
    i = 0
    # facecount variable keeps count of what face has been set
    # faces has the face names in order,
    #     uses face to acess the correct face value
    facecount = 0
    faces = ['FRONT', 'BACK', 'UP', 'DOWN', 'LEFT', 'RIGHT']
    facemat = []
    while(cap.isOpened()):
        # Capture single frame of camera feed
        ret, frame = cap.read()
        # Display the Resulting Frame
        cv2.imshow('Camera Feed', frame)
        if i == 0:
            cv2.moveWindow('Camera Feed', 70, 0)
            print('press q to quit, c to save face image')
            print('Show ' + faces[facecount] + ' face:'),
            i += 1
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print('Closing Camera Feed')
            return False
        elif key == ord('c'):
            print('Saving')
            facemat.append(frame[:, :, -1])
            facecount += 1
            print('Show ' + faces[facecount] + ' face:'),
        if facecount == 6:
            #When everything done, release the capture
            cap.release()
            cv2.destroyAllWindows()
            return facemat


