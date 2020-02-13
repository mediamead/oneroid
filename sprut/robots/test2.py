#!/usr/bin/env python3

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot

if __name__ == "__main__":
    NS = 2
    NP = 2

    tr = TensorRobot(NS, NP)
    pr = PyBulletRobot(NS, NP)
    rs = [tr, pr]

    ls = np.array([[np.pi/10,0],[np.pi/5,0]], dtype=np.float32)
    #ls = np.array([[np.pi/4,0]], dtype=np.float32)
    print("# ls=%s" % ls)

    for r in rs:
        r.step(ls)
        (p, v, u) = r.getHeadcamPVU()
        print("p=%s v=%s u=%s" % (p, v, u))

    #tr.train()

    for r in rs:
        r.close()