#!/usr/bin/env python3

"""
Try to reach red ball jumping on the ceiling
"""

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot
from hybrid_robot import HybridRobot

def sweep(N):
    r = HybridRobot(4, 4)

    #ls = np.array([[0,np.pi/8], [0,np.pi/4]], dtype=np.float32)
    #ls = np.array([[0, 0]] * NS, dtype=np.float32)

    #a0, a1, a2, a3 = np.random.rand(4) * np.pi/4
    #phis = np.array([[a0, 0], [a1, 0], [a2, 0], [a3, 0]], dtype=np.float32)
    #r.step(phis)

    line_xpos = 1. # X axis pos of green line
    line_zpos = 0.75 # Z axis pos of green line
    line_ypos = 1.5 # extent of the line along Y axis

    p_head = np.array([0.2, 0., line_zpos]) # desired position of the head
    x_head = np.array([0., 0., -1.]) # desired orientation of X axis
    y_head = np.array([0., 1., 0.]) # desired orientation of X axis

    r.pr.addHeadposMarker(p_head)
    r.pr.addGreenLine(line_xpos, line_ypos, line_zpos)

    T1 = np.array([line_xpos, -line_ypos, line_zpos])
    T2 = np.array([line_xpos, line_ypos, line_zpos])

    warming_up = True

    zs = []
    pxyzs = []
    phiss = []
    costs = []

    for i in range(N):
        T = T1 + (T2 - T1) * i / N
        print("# ----- %d/%d %s" % (i, N, T))

        z_head = T - p_head
        z_head /= np.linalg.norm(z_head)
        r.pr.setTarget(T)

        if warming_up:
            print("# warming up")
            for _ in range(25):
                cost = r.tr.model.homing_pxyz(p_head, x_head, y_head, z_head)
                print("#c=%s" % (cost))
            warming_up = False
            print("# warmed up")

        for _ in range(10):
            cost = r.tr.model.homing_pxyz(p_head, x_head, y_head, z_head)
            phis = r.tr.model.get_phis()
            pxyz = r.tr.model.get_pxyz()

            #print("pxyz, phis (got from homing)=%s" % pxyz, phis)
            #r.tr.model.set(phis)
            #print("phis (read from model vars)=%s" % phis)

            r.pr.step(phis)
            r.pr.getCameraImage()
            #r.check()

            #print("#c=%s" % (cost))

            if cost[0] <= 1e-3:
                break


        print("z_head=%s pxyz=%s phis=%s cost=%s" % (z_head, pxyz, phis.flatten(), cost))

        z_head.flatten()
        pxyzs.append(pxyz)
        phiss.append(phis)
        costs.append(cost)

    r.close()

    return zs,pxyzs,phiss,costs

if __name__ == "__main__":
    sweep(3)