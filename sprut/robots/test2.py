#!/usr/bin/env python3

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot

if __name__ == "__main__":
    NS = 2
    NP = 2

    tr = TensorRobot(NS, NP)
    pr = PyBulletRobot(NS, NP)

    ls = np.array([[np.pi/10,0],[np.pi/5,0]], dtype=np.float32)

    tr.step(ls)
    pr.step(ls)

    tr_pvu = tr.getHeadcamPVU()
    pr_pvu = pr.getHeadcamPVU()
    
    tr_p = tr_pvu[0].flatten()
    pr_p = np.array(list(pr_pvu[0]))
    err_p = np.linalg.norm(tr_p - pr_p)
    print("err_p=%f p=%s p=%s" % (err_p, tr_p, pr_p))

    #tr.train()

    tr.close()
    pr.close()