#!/usr/bin/env python3

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot

if __name__ == "__main__":
    NS = 1
    NP = 1

    tr = TensorRobot(NS, NP)
    pr = PyBulletRobot(NS, NP, render=True)

    #ls = np.array([[0,np.pi/8], [0,np.pi/4]], dtype=np.float32)
    ls = np.array([[np.pi/10, 0], [-np.pi/3, 0]], dtype=np.float32)
    #ls = np.array([[0, 0]] * NS, dtype=np.float32)

    tr.step(ls)
    pr.step(ls)

    tr_pvu = tr.getHeadcamPVU()
    pr_pvu = pr.getHeadcamPVU()
    
    tr_p = tr_pvu[0].flatten()
    pr_p = np.array(list(pr_pvu[0]))
    err_p = np.linalg.norm(tr_p - pr_p)
    print("err_p=%f p=%s p=%s" % (err_p, tr_p, pr_p))

    tr_v = tr_pvu[1].flatten()
    pr_v = np.array(list(pr_pvu[1]))
    err_v = np.linalg.norm(tr_v - pr_v)
    print("err_v=%f tr_v=%s pr_v=%s" % (err_v, tr_v, pr_v))

    # tr_u = tr_pvu[2].flatten()
    # pr_u = np.array(list(pr_pvu[2]))
    # err_u = np.linalg.norm(tr_u - pr_u)
    # print("err_u=%f tr_u=%s pr_u=%s" % (err_u, tr_u, pr_u))

    #tr.close()
    #pr.close()

    pr._print_joints_pos()

    while True:
        pr.step(ls)