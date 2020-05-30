#!/usr/bin/env python3

import pybullet as p
import numpy as np

from pybullet_robot import PyBulletRobot, W, H

import cv2
import time
import pandas as pd 

#from camorn import set_headcam_params, get_horizon_bank

if __name__ == "__main__":
    gui = True
    r = PyBulletRobot(4, 4, render=gui)

    good_phiss = pd.read_csv("goodphis-0.csv")
    print(good_phiss.values.shape)

    for i in range(1, good_phiss.shape[0]):
        phis = good_phiss.values[i,1:]
        phis = phis.reshape(-1, 2)
        r.step(phis)
        r.getCameraImage()
        time.sleep(1)
