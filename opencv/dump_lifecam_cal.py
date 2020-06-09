"""
This scripts shows calibration data, assuming it was taken with Microsoft Lifecam Studio HD at 1920x1080
"""
import sys
import cv2
import numpy as np

def showCalibration(PHY_W, PHY_H, cal_file):
    with np.load(cal_file) as CAL:
        mtx, dist, cal_w, cal_h = [CAL[i] for i in ('mtx', 'dist', 'cal_w', 'cal_h')]

    print("mtx=%s" % mtx)
    print("dist=%s" % dist)
    print("resolution=%dx%d" % (cal_w, cal_h))
    
    #newcameramtx = cv2.getOptimalNewCameraMatrix(mtx, dist, (W, H), 1, (W, H))
    values = cv2.calibrationMatrixValues(mtx, (cal_w, cal_h), PHY_W, PHY_H)
    print("fovx\t\t%f\nfovy\t\t%f\nfocalLength\t%f\nprincipalPoint\t%s\naspectRatio\t%f" % values)

# https://www.astrobin.com/gear/9596/microsoft-lifecam-studio-hd/
showCalibration(5.85, 3.27, sys.argv[1])