import cv2
import numpy as np

drawing = False  # true if mouse is pressed
ix, iy = -1, -1
i = 0  # used to position the windows once


# mouse callback function
def draw_circle(event, x, y, flags, param):
    global ix, iy, drawing

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing is True:
            cv2.circle(draw_window, (x, y), 5, (b, g, r), -1)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.circle(draw_window, (x, y), 5, (b, g, r), -1)


# function to pass back value
def nothing(x):
    pass

# Create a black image, display in a window
colourwindow = np.zeros((200, 400, 3), np.uint8)
cv2.namedWindow('colour')

# Create trackbars for colour change
cv2.createTrackbar('R', 'colour', 0, 255, nothing)
cv2.createTrackbar('G', 'colour', 0, 255, nothing)
cv2.createTrackbar('B', 'colour', 0, 255, nothing)

# create window for painting in
draw_window = np.zeros((512, 512, 3), np.uint8)
cv2.namedWindow('Paint Window')
cv2.setMouseCallback('Paint Window', draw_circle)


# main loop where we show the windows and exit with esc
while(1):
    cv2.imshow('Paint Window', draw_window)
    cv2.imshow('colour', colourwindow)
    if i == 0:
        cv2.moveWindow('Paint Window', 66, 0)
        cv2.moveWindow('colour', 600, 0)
    # Break while loop when esc is pressed
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of three trackbars
    r = cv2.getTrackbarPos('R', 'colour')
    g = cv2.getTrackbarPos('G', 'colour')
    b = cv2.getTrackbarPos('B', 'colour')
    colourwindow[:] = [b, g, r]


cv2.destroyAllWindows()