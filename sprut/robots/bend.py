#!/usr/bin/env python3

#import pybullet as p
import numpy as np

import cv2
#import matplotlib.pyplot as plt

from sprut_robot import Robot, NJ, H, W

if __name__ == "__main__":
    np.set_printoptions(precision=3)

    fourcc = cv2.VideoWriter_fourcc(*'MP42')
    if False:
        vout_headcam = cv2.VideoWriter('bend-headcam.avi', fourcc, 10, (W, H))
        vout_cam_y = cv2.VideoWriter('bend-cam-y.avi', fourcc, 10, (W, H))
    else:
        vout_headcam = None
        vout_cam_y = None

    r = Robot(render=True)
    r.setTarget([1, 0, 1])

    A0_START = 0
    A0_END = -np.pi/3
    A1_START = np.pi/3
    A1_END = np.pi
    ERRU_MAX = np.pi/20

    DA0 = (A0_END - A0_START)/50
    DA1 = (A1_END - A1_START)/250

    a0 = A0_START
    a1 = A1_START
    phis = np.zeros(2*NJ)

    phis[1] = -np.pi/4
    phis[3] = phis[5] = np.pi/4
    phis[7] = -np.pi/8

    while a0 >= A0_END:
        while True:
            phis[0] = phis[2] = a0/2
            phis[4] = phis[6] = a1/2

            r.step(phis)
            cam_p, cam_v, cam_u = r.getHeadcamPVU()
            err_ux = cam_u[0]
            print("# %f %f %s" % (a0, a1, cam_u))
            if err_ux >= 0: break

            a1 += DA1
            assert (a1 < A1_END)
        phis[7] -= -np.pi/200

        print("%.3f %.3f %.3f %.3f" % (a0, a1, cam_p[2], cam_p[0]))
        img = r.getCameraImage()

        if vout_headcam:
            vout_headcam.write(img)

        if vout_cam_y:
            imgY = r.getCameraImage(([0, 0.5, 0.5], [0, -1, 0], [0, 0, 1])) # Y
            vout_cam_y.write(imgY)

        a0 += DA0

    if vout_headcam:
        vout_headcam.release()
    if vout_cam_y:
        vout_cam_y.release()
