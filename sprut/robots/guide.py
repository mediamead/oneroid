#!/usr/bin/env python3

import pybullet as p
import numpy as np

import cv2
import matplotlib.pyplot as plt

from sprut_robot import Robot, NJ, H, W

TR = 3

if __name__ == "__main__":
    gui = False
    gui2 = False

    #t = 0
    sps = 240
    #timeStep = 1./sps # default Bullet's timestep

    #LOGFILE = "log10s.txt"
    #logf = open(LOGFILE, "w")

    DESIRED_CAM_Z = 1
    DESIRED_CAM_PHI = 45
    DESIRED_CAM_THETA = 0

    r = Robot(render=gui)

    if gui:
        s_quit = p.addUserDebugParameter("quit", 0, 1, 0)
        s_cam_phi = p.addUserDebugParameter("cam_phi", -90, 90, DESIRED_CAM_PHI)
        s_cam_theta = p.addUserDebugParameter("cam_theta", -90, 90, DESIRED_CAM_THETA)
        s_cam_z = p.addUserDebugParameter("cam_z", 0, 3, DESIRED_CAM_Z)

    if gui2:
        plt.ion()
        img = [[0,] * H*2] * W*2
        image = plt.imshow(img, interpolation='none', animated=True, label="blah")
        ax = plt.gca()
    
    if True:
        fourcc = cv2.VideoWriter_fourcc(*'MP42')
        vout = cv2.VideoWriter('guide.avi', fourcc, sps/10, (W, H))
    else:
        vout = None

    # Target and manipulator are both NNE
    r.setTarget([1.5, 1.5, 1])

    cam_phi0 = None
    cam_theta0 = None
    desired_cam_z0 = None

    phis = np.zeros(NJ * 2)
    DPHI = np.pi / 180 # one degree steps

    DPHIS = [-DPHI, 0, DPHI]
    DPHISS = []
    for dphi0 in DPHIS:
        _DPHIS = np.zeros(NJ * 2)
        _DPHIS[0] = phis[0] + dphi0/2
        _DPHIS[2] = phis[2] + dphi0/2
        for dphi1 in DPHIS:
            _DPHIS[1] = phis[1] + dphi1/2
            _DPHIS[3] = phis[3] + dphi1/2
    #        for dphi2 in DPHIS:
    #            _DPHIS[2] = phis[2] + dphi2
    #            for dphi3 in DPHIS:
    #                _DPHIS[3] = phis[3] + dphi3
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

    r.step(phis)

    while not gui or (p.readUserDebugParameter(s_quit) == 0):
        pose_changed = False
        target_changed = False

        if gui:
            # place target where user tells us
            cam_phi = p.readUserDebugParameter(s_cam_phi) / 180 * np.pi
            cam_theta = p.readUserDebugParameter(s_cam_theta) / 180 * np.pi
            desired_cam_z = p.readUserDebugParameter(s_cam_z)
        else:
            cam_phi = DESIRED_CAM_PHI
            cam_theta = DESIRED_CAM_THETA
            desired_cam_z = DESIRED_CAM_Z

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

        def get_best_phis():
            best_phis = None
            best_err = None

            #print("=======================================================")
            for dphis in DPHISS:
                other_phis = phis + dphis

                r.step(other_phis)
                cam_p, cam_v, _cap_u = r.getCamPVU()

                err_b = r.getImbalance()
                #err_p = np.linalg.norm(cam_p - desired_cam_p)
                err_v = np.dot(desired_cam_v, cam_v)
                err = err_b * 0.1 - err_v

                #print("# phis %s => err %f" % (other_phis, err))
                if best_err is None or err < best_err:
                    best_phis = np.copy(other_phis)
                    best_err = err
                    best_err_ = (err_b, err_v)

            print("err %f (%s)" % (best_err, best_err_))
            #print("%s %s err %s | %s %s" % (desired_cam_v, cam_v, best_err, phis, best_phis))

            return best_phis, best_err

        phis, err = get_best_phis()
        r.step(phis)

        if gui or gui2 or vout:
            imgs = r.getCameraImage()

        if gui2 or vout is not None:
            #(w, h, rgba, _, _) = imgs
            rgba = np.reshape(imgs[2], (H, W, 4)).astype(np.uint8)

        if gui2:
            image.set_data(rgba)
            ax.plot([0])
            plt.pause(0.01)

        if vout is not None:
            img = cv2.merge((rgba[:,:,2], rgba[:,:,1], rgba[:,:,0])) # take BGR from RBGA
            vout.write(img)

        if err < -0.99:
            break

        #t += timeStep
        #print("t=%f oc=%s, qval=%f, done=%s" % (t, (r.dx, r.dy), r.qval, r.done))

        #print("%f %f" % (t, reward), file=logf)

    #logf.close()

if vout is not None:
    vout.release()