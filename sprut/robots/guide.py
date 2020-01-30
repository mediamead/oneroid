#!/usr/bin/env python3

import pybullet as p
import numpy as np

from sprut_robot import Robot, NJ

if __name__ == "__main__":
    #t = 0
    #sps = 240
    #timeStep = 1./sps # default Bullet's timestep

    #LOGFILE = "log10s.txt"
    #logf = open(LOGFILE, "w")

    gui = True
    r = Robot(render=gui)

    # Target and manipulator are both NNE
    r.setTarget([1.5, 1.5, 1])

    if gui:
        s_cam_phi = p.addUserDebugParameter("cam_phi", -90, 90, 0)

    cam_phi0 = None
    desired_cam_v = None

    phis = np.zeros(NJ * 2)
    #phis[0] = 0.75
    #phis[1] = 0.5
    DPHI = np.pi / 180 * 0.5 # half degree steps

    while True:
        do_step = False

        if gui:
            # place target where user tells us
            TR = 3
            cam_phi = p.readUserDebugParameter(s_cam_phi) / 180 * np.pi
            if desired_cam_v is None or cam_phi != cam_phi0:
                desired_cam_v = [np.sin(cam_phi), 0, np.cos(cam_phi)]
                desired_cam_v = np.array(desired_cam_v)
                cam_phi0 = cam_phi
                #print("desired_cam_v %s" % desired_cam_v)

                r.setTarget(desired_cam_v * TR)
                do_step = True

        #if not do_step:
        #    continue

        r.step(phis)

        def get_best_phis():
            best_phis = None
            best_err = None

            #print("=======================================================")
            other_phis = np.copy(phis)
            for dphi in [-DPHI, 0, DPHI]:
                other_phis[0] = phis[0] + dphi

                r.step(other_phis)
                cam_p, cam_v, cap_u = r.getCamPVU()
                err = np.dot(desired_cam_v, cam_v)

                #print("# phis %s => err %f" % (other_phis, err))
                if best_err is None or err > best_err:
                    best_phis = np.copy(other_phis)
                    best_err = err

            #print("BEST phis %s err %f" % (best_phis, best_err))
            #print("%s %s err %s | %s %s" % (desired_cam_v, cam_v, best_err, phis, best_phis))

            return best_phis

        phis = get_best_phis()
        #import time
        #time.sleep(0.5)
        

        #t += timeStep
        #print("t=%f oc=%s, qval=%f, done=%s" % (t, (r.dx, r.dy), r.qval, r.done))

        #print("%f %f" % (t, reward), file=logf)

    #logf.close()