#!/usr/bin/env python3

import pybullet as p
import numpy as np

from pyb.pybullet_robot import PyBulletRobot
from opencv.opencv_camera import Camera
from opencv.opencv_pose import Pose

def degree2rad(a):
    return a / 180. * np.pi

if __name__ == "__main__":
    gui = True
    r = PyBulletRobot(4, 1, render=gui)
    pose = Pose(r.W, r.H, "cal.npz", D)

    if gui:
        s_t_theta = p.addUserDebugParameter("t_theta", -90, 90, 0)
        s_t_phi = p.addUserDebugParameter("t_phi", -np.pi, np.pi, 0)

        phiSliders = []
        for i in range(r.NS):    
          for j in [0, 1]:
            title = "%d:%d" % (i, j)
            s = p.addUserDebugParameter(title, -45, 45, 0)
            phiSliders.append((i, j,s))
    else:
        phis = np.zeros((r.NS, 2), dtype=np.float32)

    old_t_theta, old_t_phi = None, None
    old_phis = None
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

        if not do_step:
            continue

        r.step(phis)
        
        img = r.getCameraImage()
