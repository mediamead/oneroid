"""
Grabs frames from the camera and displays it (half size) with axes drawn on top
"""

import sys
import cv2
import numpy as np
import glob

from opencv_camera import Camera
from opencv_pose import Pose
from opencv_tools import resize, save, init_argparser, run_argparser

np.set_printoptions(linewidth=100, formatter={'float_kind': "{:6.3f}".format})

parser = init_argparser(cal_required=True)
args, params = run_argparser(parser)

cam = Camera(args.cam_device, (1920, 1080))
pose = Pose(cam.W, cam.H, args.cal_file, params['D'])

m0 = None

print("### Press 's' to save the frame, 'q' to quit")

x1 = int(cam.W / 2)
x2 = cam.W - 1
y1 = cam.H - 1
y2 = int(cam.H / 2)

while True:
    img = cam.read()
    #img = cv2.rotate(img, cv2.ROTATE_180)

    retval, rvecs, tvecs, corners = pose.findChessboardRTVecs(img)
    if retval:
        img2 = pose.drawAxes(img, rvecs, tvecs, corners)

        cv2.line(img2, (x1, 0), (x1, y1), (255, 255, 255))
        cv2.line(img2, (0, y2), (x2, y2), (255, 255, 255))

        d = np.sqrt(np.sum(tvecs**2))
        m = np.concatenate((np.array([d]), tvecs.ravel(), rvecs.ravel()))
        if m0 is not None:
            m -= m0
        print("%10.3f | %30s | %30s" % (m[0], m[1:4], m[4:7]))
    else:
        img2 = img

    img2 = resize(img2, 0.5)
    cv2.imshow('img', img2)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"): 
         break
    elif k == ord('s'):
        save(img)
    elif k == ord(" "):
        m0 = m

cam.close()
