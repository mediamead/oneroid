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

  def check(self):
    (tr_p, tr_v, tr_u) = self.tr.getHeadcamPVU()
    (pr_p, pr_v, pr_u) = self.pr.getHeadcamPVU()
    
    err_p = np.linalg.norm(tr_p - pr_p)
    err_v = np.linalg.norm(tr_v - pr_v)

    print("err_p=%f p=%s p=%s" % (err_p, tr_p, pr_p))
    print("err_v=%f tr_v=%s pr_v=%s" % (err_v, tr_v, pr_v))

  def close(self):
    self.tr.close()
    self.pr.close()

    #pr._print_joints_pos()

if __name__ == "__main__":

    r = HybridRobot(1, 1)
    phis = np.array([0., np.pi/4], dtype=np.float32)
    print("phis=%s" % phis)
    r.tr.model.set(phis)
    phis = r.tr.model.get_phis()
    print("phis=%s" % phis)

    r.pr.step(phis)
    r.pr.getCameraImage()
    r.check()

    while True: r.pr.stepSimulation() # idle loop