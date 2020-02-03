#!/usr/bin/env python3

import pybullet as p
import numpy as np

from sprut_robot import Robot, NJ

if __name__ == "__main__":
    #LOGFILE = "log10s.txt"
    #logf = open(LOGFILE, "w")

    gui = True
    r = Robot(render=gui)

    # Target and manipulator are both NNE
    r.setTarget([1.5, 1.5, 1])

    if gui:
        s_t_phi = p.addUserDebugParameter("t_phi", -1.5, 1.5, 0)

        phiSliders = []
        for i in range(NJ * 2):
            title = "%d:%d" % (i / 2, i % 2)
            s = p.addUserDebugParameter(title, -1.5, 1.5, 0)
            phiSliders.append(s)
    else:
        phis = []
        for i in range(NJ * 2):
            phis.append(0)

    old_t_phi = None
    old_phis = None
    while True:
        do_step = False
        
        if gui:
            TR = 3
            t_phi = p.readUserDebugParameter(s_t_phi)
            if old_t_phi is None or t_phi != old_t_phi:
                t_x = TR * np.sin(t_phi)
                t_y = 0
                t_z = TR * np.cos(t_phi)

                r.setTarget([t_x, t_y, t_z])
                old_t_phi = t_phi

                do_step = True

            phis = []
            for s in phiSliders:
                phi = p.readUserDebugParameter(s)
                phis.append(phi)

            #print("%s <=> %s" % (old_phis, phis))
            if old_phis is None or not np.array_equal(phis, old_phis):
                old_phis = phis

                do_step = True

        if not do_step:
            continue

        r.step(phis)
        
        imb = r.getImbalance()
        print("imb %s" % imb)

        #cam_p, cam_v, _cap_u = r.getCamPVU()
        #print("cam_p %s, cam_v %s" % (cam_p, cam_v))

        r.getCameraImage()
        #print("oc=%s, qval=%f, done=%s" % ((r.dx, r.dy), r.qval, r.done))

        #print("%f %f" % (t, reward), file=logf)

    #logf.close()