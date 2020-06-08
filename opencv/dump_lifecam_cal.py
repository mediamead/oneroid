"""
This scripts shows calibration data, assuming it was taken with Microsoft Lifecam Studio HD at 1920x1080
"""
import sys
import cv2
import numpy as np

def showCalibration(W, H, PHY_W, PHY_H, cal_file):
    with np.load(cal_file) as CAL:
        mtx, dist = [CAL[i] for i in ('mtx', 'dist')]

    #newcameramtx = cv2.getOptimalNewCameraMatrix(mtx, dist, (W, H), 1, (W, H))
    values = cv2.calibrationMatrixValues(mtx, (W,H), PHY_W, PHY_H)
    print(values)
    print("fovx\t\t%f\nfovy\t\t%f\nfocalLength\t%f\nprincipalPoint\t%s\naspectRatio\t%f" % values)

# https://www.astrobin.com/gear/9596/microsoft-lifecam-studio-hd/
showCalibration(1280, 720, 5.85, 3.27, sys.argv[1])