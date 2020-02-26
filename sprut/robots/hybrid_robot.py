#!/usr/bin/env python3

"""
Try to reach red ball jumping on the ceiling
"""

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot

class HybridRobot(object):
  def __init__(self, NS, NP):
    self.tr = TensorRobot(NS, NP)
    self.pr = PyBulletRobot(NS, NP, render=True)

  #def step(self, phis):
  #  self.tr.step(phis)
  #  self.pr.step(phis)

  def check(self): #, pxyz=None):
#    if pxyz is not None:
#      (tr_p, tr_z, tr_u) = pxyz[0], pxyz[3], pxyz[1]
#    else:
    (tr_p, tr_z, tr_u) = self.tr.getHeadcamPVU()
    (pr_p, pr_z, pr_u) = self.pr.getHeadcamPVU()
    
    err_p = np.linalg.norm(tr_p - pr_p)
    err_z = np.linalg.norm(tr_z - pr_z)

    print("err_p=%f p=%s p=%s" % (err_p, tr_p, pr_p))
    print("err_z=%f tr_z=%s pr_z=%s" % (err_z, tr_z, pr_z))

  def close(self):
    self.tr.close()
    self.pr.close()

    #pr._print_joints_pos()

if __name__ == "__main__":

    r = HybridRobot(4, 4)
    phis = np.array([[np.pi/10, np.pi/16], [np.pi/8, np.pi/20]] * 2, dtype=np.float32)

    print("phis (to be assigned to model vars)=%s" % phis)
    r.tr.model.set(phis)
    phis = r.tr.model.get_phis()
    print("phis (read from model vars)=%s" % phis)

    r.pr.step(phis)
    r.pr.getCameraImage()
    r.check()

    while True: r.pr.stepSimulation() # idle loop