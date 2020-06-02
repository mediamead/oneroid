"""
Finds chessboard in the frame and draws lines/circles on it.
"""

import numpy as np
import cv2
import glob

# https://gitmemory.com/issue/opencv/opencv/15712/614516890

cap = cv2.VideoCapture(1)

def chessdet(img_rgb):
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    retval, corners = cv2.findChessboardCornersSB(img, (14, 9), flags=flags)
    return cv2.drawChessboardCorners(img_rgb, (14, 9), corners, retval)

while True:
    ret, img = cap.read()
    if ret == False:
        print("Failed to retrieve frame")
        break 
    cv2.imshow('feed', img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img2 = chessdet(img)
    cv2.imshow('img', img2)

    k = cv2.waitKey(30) & 0xff
    if k == ord('q'):
        break

cap.release() 
cv2.destroyAllWindows()
