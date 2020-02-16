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

    line_xpos = 1. # X axis pos of green line
    line_zpos = 0.7 # Z axis pos of green line
    line_ypos = 1. # extent of the line along Y axis

    p_head = np.array([0., 0., line_zpos]) # desired position of the head
    x_head = np.array([0., 0., -1.]) # desired orientation of X axis

    r.pr.addHeadposMarker(p_head)
    r.pr.addGreenLine(line_xpos, line_ypos, line_zpos)

    while True:
        print("-" * 40)
        p_target = np.array([line_xpos, line_ypos - 2*line_ypos*np.random.rand(), line_zpos])
        z_head = p_target - p_head
        r.pr.setTarget(p_target)

        for _ in range(10):
            r.tr.model.homing_pzx(p_head, z_head, x_head)
            phis = r.tr.model.get_phis()

            #print("pxyz, phis (got from homing)=%s" % pxyz, phis)
            #r.tr.model.set(phis)
            #print("phis (read from model vars)=%s" % phis)

            r.pr.step(phis)
            r.pr.getCameraImage()
            r.check()