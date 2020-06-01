#!/usr/bin/env python3

import pybullet as p
import numpy as np

from pybullet_robot import PyBulletRobot, W, H

import cv2
import time
import sys, glob
import pandas as pd 

#from camorn import set_headcam_params, get_horizon_bank

gui = True
r = PyBulletRobot(3, 1, render=gui)

good_phiss = None
for fname in glob.glob(sys.argv[1]):
    df = pd.read_csv(fname)
    if good_phiss is None:
        good_phiss = df
    else:
        good_phiss = pd.concat([good_phiss, df])
print(good_phiss.values.shape)

gps = []
for i in range(1, good_phiss.shape[0]):
    phis = good_phiss.values[i,1:]
    phis = phis.reshape(-1, 2)

    r.step(phis)

    if phis[0][0] < 0:
        gps.append(phis)

print(len(gps))

for gp in gps:
    r.step(gp)
    cam_p, cam_v, cam_u = r.getHeadcamPVU()

    r.getCameraImage()
    time.sleep(1)
