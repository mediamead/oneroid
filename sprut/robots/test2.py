#!/usr/bin/env python3

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot

if __name__ == "__main__":
    rs = []
    rs.append(TensorRobot())
    rs.append(PyBulletRobot())

    ls = np.array([[np.pi/4,0],[0,0],[0,0],[0,0]], dtype=np.float32)
    print("# ls=%s" % ls)

    for r in rs:
        r.step(ls)
        (p, v, u) = r.getHeadcamPVU()
        print("p=%s v=%s u=%s" % (p, v, u))

    for r in rs:
        r.close()