#!/usr/bin/env python3

import sprut
import numpy as np

if __name__ == "__main__":
    t = 0
    sps = 240
    timeStep = 1./sps # default Bullet's timestep

    LOGFILE = "sweep.txt"
    logf = open(LOGFILE, "w")

    gui = False
    r = sprut.Robot(render=gui)

    phi_values = np.arange(-np.pi/4, np.pi/4, 0.025)
    phis = np.zeros(sprut.NJ * 2)
    for t_phi in phi_values:
        TR = 3
        t_x = TR * np.sin(t_phi)
        t_y = 0
        t_z = TR * np.cos(t_phi)
        r.setTarget([t_x, t_y, t_z])

        for phi0 in phi_values:
            phis[0] = phi0
            r.step(phis)
            print("%f %f %f" % (t_phi, phi0, r.qval), file=logf)

        t += timeStep
        print("# t_phi %f done" % (t_phi))
        logf.flush()

    logf.close()
