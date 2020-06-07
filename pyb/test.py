#!/usr/bin/env python3

import pybullet as p
import numpy as np
import cv2

from pyb.pybullet_robot import PyBulletRobot
from opencv.opencv_pose import Pose
from opencv.opencv_tools import resize

def degree2rad(a):
    return a / 180. * np.pi

if __name__ == "__main__":
    gui = True
    r = PyBulletRobot(4, 1, render=gui)
    pose = Pose(r.W, r.H, "cal.npz", r.D)

    phis = np.zeros((r.NS, 2), dtype=np.float32)
    phis[:,0] = 90/4

    if gui:
        # configure GUI sliders
        s_t_theta = p.addUserDebugParameter("t_theta", -90, 90, 0)
        s_t_phi = p.addUserDebugParameter("t_phi", -np.pi, np.pi, 0)

        phiSliders = []
        for i in range(r.NS):    
          for j in [0, 1]:
            title = "%d:%d" % (i, j)
            s = p.addUserDebugParameter(title, -45, 45, phis[i, j])
            phiSliders.append((i, j,s))

    old_t_theta, old_t_phi = None, None
    old_phis = None
    m0 = None

    while True:
        do_step = False
        
        if gui:
            TR = 3
            t_theta = degree2rad(p.readUserDebugParameter(s_t_theta))
            t_phi = degree2rad(p.readUserDebugParameter(s_t_phi))
            if old_t_theta is None or old_t_phi is None or t_phi != old_t_phi or t_theta != old_t_theta:
                t_x = TR * np.sin(t_theta) * np.cos(t_phi)
                t_y = TR * np.sin(t_theta) * np.sin(t_phi)
                t_z = TR * np.cos(t_theta)

                r.setTarget([t_x, t_y, t_z])
                old_t_theta, old_t_phi = t_theta, t_phi
                do_step = True

            phis = np.zeros((r.NS, 2), dtype=np.float32)
            for (i, j, s) in phiSliders:
                phi = degree2rad(p.readUserDebugParameter(s))
                phis[i, j] = phi

            if old_phis is None or not np.array_equal(phis, old_phis):
                old_phis = phis
                do_step = True

        #if not do_step:
        #    continue

        r.step(phis)
        
        img = r.getCameraImage()

        retval, rvecs, tvecs, corners = pose.findChessboardRTVecs(img)
        if retval:
            img = pose.drawAxes(img, rvecs, tvecs, corners)

            d = np.sqrt(np.sum(tvecs**2))
            m = np.concatenate((np.array([d]), tvecs.ravel(), rvecs.ravel()))
            if m0 is not None:
                m -= m0
            print("%10.3f | %30s | %30s" % (m[0], m[1:4], m[4:7]))

        img = resize(img, 0.5)
        cv2.imshow('img', img)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"): 
            break 
        elif k == ord(" "):
            m0 = m
