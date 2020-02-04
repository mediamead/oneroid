#!/usr/bin/env python3

import pybullet as p
import numpy as np

import cv2
import matplotlib.pyplot as plt

from sprut_robot import Robot, NJ, H, W

TR = 3

if __name__ == "__main__":
    gui = True
    gui2 = True
    auto_retarget = False

    #t = 0
    sps = 240
    #timeStep = 1./sps # default Bullet's timestep

    #LOGFILE = "log10s.txt"
    #logf = open(LOGFILE, "w")

    DESIRED_CAM_PHI = 45 # np.pi / 4
    DESIRED_CAM_THETA = 0

    r = Robot(render=gui)

    if gui:
        s_quit = p.addUserDebugParameter("quit", 0, 1, 0)
    if gui and not auto_retarget:
        s_cam_phi = p.addUserDebugParameter("cam_phi", -90, 90, DESIRED_CAM_PHI)
        s_cam_theta = p.addUserDebugParameter("cam_theta", -90, 90, DESIRED_CAM_THETA)
    
    if True:
        fourcc = cv2.VideoWriter_fourcc(*'MP42')
        vout = cv2.VideoWriter('guide.avi', fourcc, sps/10, (W*2, H*2))
    else:
        vout = None

    # ===================================================================

    # Target and manipulator are both NNE
    r.setTarget([1.5, 1.5, 1])

    target_phi0 = None
    target_theta0 = None

    phis = np.zeros(NJ * 2)
    DPHI = (np.pi / 180) * 1 # steps size, in degrees

    DPHIS = [-DPHI, 0, DPHI]
    DPHISS = []

    for dphi0 in DPHIS:
        _DPHIS = np.zeros(NJ * 2)
        _DPHIS[0] = phis[0] + dphi0
        for dphi1 in DPHIS:
            _DPHIS[1] = phis[1] + dphi1
            for dphi2 in DPHIS:
                _DPHIS[2] = phis[2] + dphi2
                for dphi3 in DPHIS:
                    _DPHIS[3] = phis[3] + dphi3
                    for dphi4 in DPHIS:
                        _DPHIS[4] = phis[4] + dphi4
                        for dphi5 in DPHIS:
                            _DPHIS[5] = phis[5] + dphi5
                            for dphi6 in DPHIS:
                                _DPHIS[6] = phis[6] + dphi6
                                for dphi7 in DPHIS:
                                    _DPHIS[7] = phis[7] + dphi7
                                    DPHISS.append(np.copy(_DPHIS))
    #print(DPHISS)

    if not gui:
        target_phi = DESIRED_CAM_PHI
        target_theta = DESIRED_CAM_THETA

    if auto_retarget:
        err = None

    while not gui or (p.readUserDebugParameter(s_quit) == 0):
        # apply phis
        r.step(phis)

        # update current views in gui, gui2, and vout
        if gui is not None or gui2 is not None or vout is not None:
            img = r.getCameraImage()

        if gui2 is not None or vout is not None:
            imgX = r.getCameraImage(([2, 0, 1], [-1, 0, 0], [0, 0, 1])) # X
            imgY = r.getCameraImage(([0, 2, 1], [0, -1, 0], [0, 0, 1])) # Y
            imgZ = r.getCameraImage(([0, 0, 4], [0, 0, -1], [1, 0, 0])) # Z

            img = np.concatenate((
                np.concatenate((img, imgX), axis=1),
                np.concatenate((imgY, imgZ), axis=1)
            ), axis=0)

        if gui2:
            cv2.imshow("img", img)
            key = cv2.waitKey(1)
            #print("WAITKEY=>%s<" % key)
            if key == ord('q'):
                break
            elif key == ord(' '):
                err = None

        if vout is not None:
            vout.write(img)

        target_changed = False
        if gui and not auto_retarget:
            # place target where user tells us
            target_phi = p.readUserDebugParameter(s_cam_phi) / 180 * np.pi
            target_theta = p.readUserDebugParameter(s_cam_theta) / 180 * np.pi

        if auto_retarget and (err is None or err[0] < -3.8):
            # retarget
            target_phi = np.random.uniform(low=0, high=np.pi/2)
            target_theta = np.random.uniform(low=np.pi/6, high=np.pi/3)
            print("retargeted to %f/%f" % (target_phi, target_theta))

        if (target_phi0 is None) or (target_theta0 is None) or (target_phi != target_phi0) or (target_theta != target_theta0):
            target_phi0 = target_phi
            target_theta0 = target_theta
            #desired_cam_v = np.array([np.cos(cam_phi), np.sin(cam_phi), 0]) # XY plane
            #desired_cam_v = np.array([np.sin(cam_phi), 0, np.cos(cam_phi)]) # ZX plane
            t_x = np.sin(target_phi)*np.cos(target_theta)
            t_y = np.cos(target_phi)*np.sin(target_theta)
            t_z = np.cos(target_phi)
            target = np.array([t_x, t_y, t_z]) * TR
            r.setTarget(target)
            target_changed = True

        def get_best_phis():
            best_phis = None
            best_err = None

            cam_p = r.getHeadcamPVU()[0]
            desired_cam_v = target - cam_p
            desired_cam_v /= np.linalg.norm(desired_cam_v)

            #print("=======================================================")
            for dphis in DPHISS:
                other_phis = phis + dphis

                r.step(other_phis)
                (cam_v, cam_u) = r.getHeadcamPVU()[1:3]

                err_b = r.getImbalance()
                err_u = np.dot([0,0,1], cam_u)
                err_v = np.dot(desired_cam_v, cam_v)
                err = err_b * 0.025 - err_v - 3*err_u

                #print("# phis %s => err %f" % (other_phis, err))
                if best_err is None or err < best_err[0]:
                    best_phis = np.copy(other_phis)
                    best_err = [err, err_b, err_v, err_u]

            print("best_err: %s" % (best_err))
            #print("%s %s err %s | %s %s" % (desired_cam_v, cam_v, best_err, phis, best_phis))

            return best_phis, best_err

        phis, err = get_best_phis()

        #t += timeStep
        #print("t=%f oc=%s, qval=%f, done=%s" % (t, (r.dx, r.dy), r.qval, r.done))

        #print("%f %f" % (t, reward), file=logf)

    #logf.close()

if vout is not None:
    vout.release()