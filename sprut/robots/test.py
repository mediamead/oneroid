#!/usr/bin/env python3

import pybullet as p
import numpy as np

from pybullet_robot import PyBulletRobot, W, H

import tensorflow as tf

def get_cam_pvu(alpha, beta):
    # inclination θ, azimuth φ
    theta = np.pi/2 - beta
    phi = alpha
    cam_vx = np.sin(theta) * np.cos(phi)
    cam_vy = np.sin(theta) * np.sin(phi)
    cam_vz = np.cos(theta)
       
    theta = -beta
    phi = alpha
    cam_ux = np.sin(theta) * np.cos(phi)
    cam_uy = np.sin(theta) * np.sin(phi)
    cam_uz = np.cos(theta)
    
    cam_v = [cam_vx, cam_vy, cam_vz]
    cam_u = [cam_ux, cam_uy, cam_uz]
    cam_p = [0.23, 0.0, 0.67]
    
    return cam_p, cam_v, cam_u

if __name__ == "__main__":
    gui = True
    r = PyBulletRobot(4, 1, render=gui)

    if gui:
        s_t_theta = p.addUserDebugParameter("t_theta", -np.pi/2, np.pi/2, 0)
        s_t_phi = p.addUserDebugParameter("t_phi", -np.pi, np.pi, 0)

    alpha = 0
    dalpha = 0.01

    model = tf.keras.models.load_model('pose2phis')

    old_t_theta, old_t_phi = None, None
    while True:
        
        if gui:
            TR = 3
            t_theta = p.readUserDebugParameter(s_t_theta)
            t_phi = p.readUserDebugParameter(s_t_phi)
            if old_t_theta is None or old_t_phi is None or t_phi != old_t_phi or t_theta != old_t_theta:
                t_x = TR * np.sin(t_theta) * np.cos(t_phi)
                t_y = TR * np.sin(t_theta) * np.sin(t_phi)
                t_z = TR * np.cos(t_theta)

                r.setTarget([t_x, t_y, t_z])
                old_t_theta, old_t_phi = t_theta, t_phi

                do_step = True

        cam_p, cam_v, cam_u = get_cam_pvu(alpha, 0)
        pose = np.array([cam_p + cam_v + cam_u], dtype=np.float32)
        phis = model.predict(pose)
        r.step(phis.reshape(-1, 2))

        offc = r.getOffCenter()
        if offc is None:
            continue

        if offc[0] > 0:
            alpha += dalpha
        elif offc[0] < 0:
            alpha -= dalpha