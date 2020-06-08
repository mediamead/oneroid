#!/usr/bin/env python3

"""
This script sweeps PyBullet robot to capture calibration images.
Saves images in the current directory.
"""

import cv2

import pybullet as p
import numpy as np
from opencv.opencv_tools import save

from pybullet_robot import PyBulletRobot

def calibrate(phi1x, phi1y, phi2x, phi2y):
    phis = np.array([[-np.pi/8, 0], [phi1x, phi1y], [phi2x, phi2y/2], [phi2x, phi2y/2]])
    r.step(phis.reshape(-1, 2))

    img = r.getCameraImage()

    cv2.imshow('img', img)
    cv2.waitKey(1)

    save(img)

if __name__ == "__main__":
    gui = True
    r = PyBulletRobot(4, 4, render=gui)

    phisx_sweep = [np.pi/4.5, np.pi/5, np.pi/5.5]
    phisy_sweep = [-np.pi/8, 0, np.pi/8]

    for phi1x in phisx_sweep:
        for phi1y in phisy_sweep:
            for phi2x in phisx_sweep:
                for phi2y in phisy_sweep:
                    calibrate(phi1x, phi1y, phi2x, phi2y)
