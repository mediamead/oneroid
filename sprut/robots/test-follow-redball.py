#!/usr/bin/env python3

"""
Try to reach red ball jumping on the ceiling
"""

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot
from hybrid_robot import HybridRobot

if __name__ == "__main__":

    r = HybridRobot(4, 4)

    #ls = np.array([[0,np.pi/8], [0,np.pi/4]], dtype=np.float32)
    #ls = np.array([[0, 0]] * NS, dtype=np.float32)

    #a0, a1, a2, a3 = np.random.rand(4) * np.pi/4
    #phis = np.array([[a0, 0], [a1, 0], [a2, 0], [a3, 0]], dtype=np.float32)
    #r.step(phis)

    head_zpos = 0.7
    p_head = np.array([0., 0., head_zpos]) # position of the head is 80cm above origin
    x_head = np.array([0., 0., -1.]) # orientation of x axis should stay the same

    r.pr.addHeadposMarker(p_head)

    A = 1.

    while True:
        print("-" * 40)
        p_target = np.array([1., A/2 - A*np.random.rand(), head_zpos])
        z_head = p_target - p_head
        r.pr.setTarget(p_target)

        for _ in range(3):
            r.tr.model.homing_pzx(p_head, z_head, x_head)
            phis = r.tr.model.get_phis()

            #print("pxyz, phis (got from homing)=%s" % pxyz, phis)
            #r.tr.model.set(phis)
            #print("phis (read from model vars)=%s" % phis)

            r.pr.step(phis)
            r.pr.getCameraImage()
            r.check()