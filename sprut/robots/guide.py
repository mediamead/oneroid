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
        s_cam_theta = p.addUserDebugParameter("cam_theta", -90, 90, 0)
        s_cam_z = p.addUserDebugParameter("cam_z", 0, 3, 1)

    cam_phi0 = None
    cam_theta0 = None
    desired_cam_z0 = None

    phis = np.zeros(NJ * 2)
    DPHI = np.pi / 180 * 0.5 # half degree steps

    DPHIS = [-DPHI, 0, DPHI]
    DPHISS = []
    for dphi0 in DPHIS:
        _DPHIS = np.zeros(8)
        _DPHIS[0] = phis[0] + dphi0
        for dphi1 in DPHIS:
            _DPHIS[1] = phis[1] + dphi1
            for dphi2 in DPHIS:
                _DPHIS[2] = phis[2] + dphi2
                for dphi3 in DPHIS:
                    _DPHIS[3] = phis[3] + dphi3
                    DPHISS.append(np.copy(_DPHIS))
    print(DPHISS)

    while True:
        pose_changed = False
        target_changed = False

        if gui:
            # place target where user tells us
            TR = 3
            cam_phi = p.readUserDebugParameter(s_cam_phi) / 180 * np.pi
            cam_theta = p.readUserDebugParameter(s_cam_theta) / 180 * np.pi
            desired_cam_z = p.readUserDebugParameter(s_cam_z)

            if desired_cam_z0 is None or desired_cam_z != desired_cam_z0:
                desired_cam_z0 = desired_cam_z
                desired_cam_p = np.array([0, 0, desired_cam_z]) ## HEAD POSITION
                pose_changed = True

            if (cam_phi0 is None) or (cam_theta0 is None) or (cam_phi != cam_phi0) or (cam_theta != cam_theta0):
                cam_phi0 = cam_phi
                cam_theta0 = cam_theta
                #desired_cam_v = np.array([np.cos(cam_phi), np.sin(cam_phi), 0]) # XY plane
                #desired_cam_v = np.array([np.sin(cam_phi), 0, np.cos(cam_phi)]) # ZX plane
                t_x = np.sin(cam_phi)*np.cos(cam_theta)
                t_y = np.sin(cam_phi)*np.sin(cam_theta)
                t_z = np.cos(cam_phi)
                target = np.array([t_x, t_y, t_z]) * TR
                r.setTarget(target)
                target_changed = True

            if pose_changed or target_changed:
                desired_cam_v = target - desired_cam_p
                desired_cam_v /= np.linalg.norm(desired_cam_v)

                print("desired_cam_v %s, desired_cam_p %s" % (desired_cam_v, desired_cam_p))

        r.step(phis)

        def get_best_phis():
            best_phis = None
            best_err = None

            #print("=======================================================")
            for dphis in DPHISS:
                other_phis = phis + dphis

                r.step(other_phis)
                cam_p, cam_v, cap_u = r.getCamPVU()

                err_p = np.linalg.norm(cam_p - desired_cam_p)
                err_v = np.dot(desired_cam_v, cam_v)
                #err = err_p - err_v
                err = err_p*0.1 - err_v

                #print("# phis %s => err %f" % (other_phis, err))
                if best_err is None or err < best_err:
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