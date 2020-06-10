"""
Grabs frames from the camera and displays it (half size) with axes drawn on top
"""

import sys
import cv2
import numpy as np
import glob

from opencv_camera import Camera
from opencv_pose import Pose
from opencv_tools import resize, init_argparser, run_argparser

np.set_printoptions(linewidth=100, formatter={'float_kind': "{:6.3f}".format})

parser = init_argparser(cal_required=True)
args, params = run_argparser(parser)

cam = Camera(args.cam_device, 1920, 1080)
pose = Pose(cam.W, cam.H, args.cal_file, params['D'])

m0 = None

while True:
    img = cam.read()
    #img = cv2.rotate(img, cv2.ROTATE_180)

    retval, rvecs, tvecs, corners = pose.findChessboardRTVecs(img)
    if retval:
        img = pose.drawAxes(img, rvecs, tvecs, corners)

        d = np.sqrt(np.sum(tvecs**2))
        m = np.concatenate((np.array([d]), tvecs.ravel(), rvecs.ravel()))
        if m0 is not None:
            m -= m0
        print("%10.3f | %30s | %30s" % (m[0], m[1:4], m[4:7]))

    img2 = resize(img, 0.5)
    cv2.imshow('img', img2)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"): 
        break 
    elif k == ord(" "):
        m0 = m

cam.close()
