#!/usr/bin/env python3

import pybullet as p
import numpy as np

from pybullet_robot import PyBulletRobot, NJ, W, H

import cv2

from camorn import set_headcam_params, get_horizon_bank

if __name__ == "__main__":
    #LOGFILE = "log10s.txt"
    #logf = open(LOGFILE, "w")

    gui = True
    r = PyBulletRobot(render=gui)

    set_headcam_params(W, H)

    # Target and manipulator are both NNE
    r.setTarget([1.5, 1.5, 1])

    if gui:
        s_t_phi = p.addUserDebugParameter("t_phi", -1.5, 1.5, 0)

        phiSliders = []
        for i in range(NJ):    
          for j in [0, 1]:
            title = "%d:%d" % (i, j)
            s = p.addUserDebugParameter(title, -1.5, 1.5, 0)
            phiSliders.append((i, j,s))
    else:
        phis = np.zeros((NJ, 2), dtype=np.float32)
        #for i in range(NJ * 2):
        #phis.append(0)

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

            phis = np.zeros((NJ, 2), dtype=np.float32)
            for (i, j, s) in phiSliders:
                phi = p.readUserDebugParameter(s)
                phis[i, j] = phi

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

        img = r.getCameraImage()
        print("horizon_bank=%s" % get_horizon_bank(img))

        #print("oc=%s, qval=%f, done=%s" % ((r.dx, r.dy), r.qval, r.done))

        #print("%f %f" % (t, reward), file=logf)

    #logf.close()