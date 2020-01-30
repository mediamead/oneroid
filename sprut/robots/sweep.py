#!/usr/bin/env python3

from sprut_robot import Robot, NJ
import numpy as np

if __name__ == "__main__":
    t = 0
    sps = 240
    timeStep = 1./sps # default Bullet's timestep

    LOGFILE = "sweep"
    #logX = open("%s-x.txt" % LOGFILE, "w")
    #logY = open("%s-y.txt" % LOGFILE, "w")
    #logXY = open("%s.xy.txt" % LOGFILE, "w")

    dphi = 0.033
    logXY = open("%s.xy-%f.txt" % (LOGFILE, dphi), "w")

    gui = False
    r = Robot(render=gui)

    phi_values = np.arange(-np.pi/4, np.pi/4, dphi)
    phis = np.zeros(NJ * 2)
    for t_phi in phi_values:
        TR = 3
        t_x = TR * np.sin(t_phi)
        t_y = 0
        t_z = TR * np.cos(t_phi)
        r.setTarget([t_x, t_y, t_z])

        best_dr = None
        targets = []
        for phi in phi_values:
            phis[0] = phi
            r.step(phis)

            if r.target_found:
                targets.append((phi, r.dx, r.dy))
                if best_dr is None or r.dr < best_dr:
                    best_dr = r.dr
                    best_phi = phi

        for tgt in targets:
            #print("%f %f %f" % tgt, file=logX)
            #print("%f" % (best_phi), file=logY)

            print("%f %f %f %f -> %f" % (t_phi, tgt[0], tgt[1], tgt[2], best_phi))
            print("%f %f %f %f" % (tgt[0], tgt[1], tgt[2], best_phi), file=logXY)

        t += timeStep
        #print("# t_phi %f done" % (t_phi))
        #logX.flush()
        #logY.flush()
        logXY.flush()

    #logX.close()
    #logY.close()
    logXY.close()
