"""
TODO: refactor

Calibrates on chessboard images *.jpg from the current directory.
Expects 14x9 chessboard (see findChessboardCornersSB).
Saves calibration data into 'cal.npz'

https://docs.opencv.org/4.3.0/checkerboard_radon.png
"""

import numpy as np
import cv2
import sys, glob

# sensor size of Microsoft Lifecam Studio HD
# (from https://www.astrobin.com/gear/9596/microsoft-lifecam-studio-hd/)
PHY_W, PHY_H = 5.85, 3.27

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

flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
images = []

if len(sys.argv) > 1:
    fnames = sys.argv[1:]
else:
    fnames = glob.glob('*.jpg')

cal_w, cal_h = None, None
n_good_files = 0

print("Calibrating on %d files:" % (len(fnames)))
for fname in fnames:
    img = cv2.imread(fname)
    h, w = img.shape[:2]
    if cal_w is None or cal_h is None:
        cal_w = w
        cal_h = h
    else:
        assert((w == cal_w) and (h == cal_h))

    images.append(img)

    # Find the chess board corners
    retval, corners = cv2.findChessboardCornersSB(img, (N, M), flags=flags)
    if retval:
        print("\t%s: ok" % fname)
        n_good_files += 1
        objpoints.append(objp)
        imgpoints.append(corners)
        
        if DEBUG:
            img2 = cv2.drawChessboardCorners(img, (N, M), corners, retval)
            cv2.imshow('img', img2)
            cv2.waitKey(250)
    else:
        print("\t%s: chessboard corners not found" % fname)

print("Found %d files suitable for calibration. Resolution: %dx%d" % (n_good_files, cal_w, cal_h))

# calibrate
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (cal_w, cal_h), None, criteria)
np.savez(FNAME, retval=retval, mtx=mtx, dist=dist, cal_w=cal_w, cal_h=cal_h)

matrix_values = cv2.calibrationMatrixValues(mtx, (cal_w, cal_h), PHY_W, PHY_H)
print("Calibration data saved in '%s'. Camera matrix values:" % (FNAME))
print("\tfovx\t\t%f\n\tfovy\t\t%f\n\tfocalLength\t%f\n\tprincipalPoint\t%s\n\taspectRatio\t%f" % matrix_values)

# show undistorted calibration images
if DEBUG:
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (cal_w, cal_h), 1, (cal_w, cal_h))
    for img in images:
        undist = cv2.undistort(img, mtx, dist, None, newcameramtx)
        cv2.imshow('img', img)
        cv2.imshow('undist', undist)
        cv2.waitKey(2000)

# calculate re-projection error (have no clue what it is, see also https://stackoverflow.com/questions/11918315/does-a-smaller-reprojection-error-always-means-better-calibration)
print("Calibration error:")
tot_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    print("%s error %f" % (fnames[i], error))
    tot_error += error
print("Mean error: ", tot_error/len(objpoints))

cv2.destroyAllWindows()
