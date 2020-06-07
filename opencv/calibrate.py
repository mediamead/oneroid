"""
TODO: refactor

Calibrates on chessboard images *.jpg from the current directory.
Expects 14x9 chessboard (see findChessboardCornersSB).
Saves calibration data into 'cal.npz'

https://docs.opencv.org/4.3.0/checkerboard_radon.png
"""

import numpy as np
import cv2
import glob

N = 14
M = 9
FNAME = "cal"

DEBUG = False # set to True to see calibration process and undistorted images

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((N*M, 3), np.float32)
objp[:,:2] = np.mgrid[0:N, 0:M].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
images = []

for fname in glob.glob('*.jpg'):
    img = cv2.imread(fname)
    images.append(img)

    # Find the chess board corners
    retval, corners = cv2.findChessboardCornersSB(img, (N, M), flags=flags)
    if retval:
        print("%s: ok" % fname)
        objpoints.append(objp)
        imgpoints.append(corners)
        
        if DEBUG:
            img2 = cv2.drawChessboardCorners(img, (N, M), corners, retval)
            cv2.imshow('img', img2)
            cv2.waitKey(250)
    else:
        print("%s: no chessboard found" % fname)

# calibrate
h, w = img.shape[:2]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
np.savez(FNAME, retval=retval, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

print("Calibration data for %dx%d written to '%s'" % (w, h, FNAME))

# show undistorted calibration images
if DEBUG:
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    for img in images:
        undist = cv2.undistort(img, mtx, dist, None, newcameramtx)
        cv2.imshow('img', img)
        cv2.imshow('undist', undist)
        cv2.waitKey(2000)

# calculate re-projection error (have no clue what it is, see also https://stackoverflow.com/questions/11918315/does-a-smaller-reprojection-error-always-means-better-calibration)
tot_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error
print("mean error: ", tot_error/len(objpoints))

cv2.destroyAllWindows()
