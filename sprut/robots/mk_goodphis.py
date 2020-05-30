#!/usr/bin/env python3

import pybullet as p
import numpy as np

from pybullet_robot import PyBulletRobot, W, H

import cv2
import time
import pandas as pd 

#from camorn import set_headcam_params, get_horizon_bank

MAX_PHI = np.pi/4

if __name__ == "__main__":
    #LOGFILE = "log10s.txt"
    #logf = open(LOGFILE, "w")

    gui = True
    r = PyBulletRobot(4, 4, render=gui)

    #set_headcam_params(W, H)

    # Target and manipulator are both NNE
    r.setTarget([1.5, 1.5, 1])

    if gui:
        s_t_theta = p.addUserDebugParameter("t_theta", -np.pi/2, np.pi/2, 0)
        s_t_phi = p.addUserDebugParameter("t_phi", -np.pi, np.pi, 0)

        phiSliders = []
        for i in range(r.NS):    
          for j in [0, 1]:
            title = "%d:%d" % (i, j)
            s = p.addUserDebugParameter(title, -MAX_PHI, MAX_PHI, 0)
            phiSliders.append((i, j,s))
    else:
        phis = np.zeros((r.NS, 2), dtype=np.float32)
        #for i in range(NJ * 2):
        #phis.append(0)

    old_t_theta, old_t_phi = None, None
    old_phis = None

    NSTEPS = 10
    nsteps = 0
    tstep0 = time.time()

    good_phiss = []
    nbatch = 0
    BATCHSIZE = 10000

    while True:
        do_step = False
        
        if gui:
            TR = 1.5
            t_theta = p.readUserDebugParameter(s_t_theta)
            t_phi = p.readUserDebugParameter(s_t_phi)
            if old_t_theta is None or old_t_phi is None or t_phi != old_t_phi or t_theta != old_t_theta:
                t_x = TR * np.sin(t_theta) * np.cos(t_phi)
                t_y = TR * np.sin(t_theta) * np.sin(t_phi)
                t_z = TR * np.cos(t_theta)

                r.setTarget([t_x, t_y, t_z])
                old_t_theta, old_t_phi = t_theta, t_phi

                do_step = True

            phis = np.zeros((r.NS, 2), dtype=np.float32)
            for (i, j, s) in phiSliders:
                phi = p.readUserDebugParameter(s)
                phis[i, j] = phi

            #print("%s <=> %s" % (old_phis, phis))
            if old_phis is None or not np.array_equal(phis, old_phis):
                old_phis = phis

                do_step = True

        #if not do_step:
        #    continue

        phis = np.random.uniform(low=-MAX_PHI, high=MAX_PHI, size=(r.NS, 2))
        r.step(phis)
        
        #imb = r.getImbalance()
        #print("imb %s" % imb)
        #if do_step:
        #    offc = r.getOffCenter()
        #    print("offc %s" % str(offc))
        #    print("cam_v %s, cam_u %s" % (cam_v, cam_u))

        cam_p, cam_v, cam_u = r.getHeadcamPVU()
        #print("cam_p %s, cam_v %s" % (cam_p, cam_v))
        if cam_v[0] > 0: # the camera points towards positive X half
            if cam_u[2] > 0.995: # the camera is upright
                #print(phis, cam_p, cam_v, cam_u)
                good_phiss.append(phis.reshape(-1))
                if len(good_phiss) > BATCHSIZE:
                    pd.DataFrame(good_phiss).to_csv("goodphis-%d.csv" % nbatch)
                    good_phiss = []
                    nbatch = nbatch + 1
            else:
                continue
        else:
            continue

        #img = r.getCameraImage()
        #print("horizon_bank=%s" % get_horizon_bank(img))

        #print("oc=%s, qval=%f, done=%s" % ((r.dx, r.dy), r.qval, r.done))

        #print("%f %f" % (t, reward), file=logf)
        nsteps = nsteps + 1
        now = time.time()
        if now - tstep0 > 1: # report TPS every second
            tps = nsteps / (now - tstep0)
            print("%d %.3f" % (len(good_phiss), tps))
            tstep0 = now
            nsteps = 0

    #for good_phis in good_phiss:
    #    r.step(good_phis.reshape(-1, 2))
    #    r.getCameraImage()
    #    time.sleep(3)

    #logf.close()
