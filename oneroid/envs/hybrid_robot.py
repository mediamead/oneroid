#!/usr/bin/env python3

import numpy as np
from pybullet_robot import PyBulletRobot
from tensor_robot import TensorRobot

class HybridRobot(object):
  def __init__(self, NS, NP):
    self.tr = TensorRobot(NS, NP)
    self.pr = PyBulletRobot(NS, NP, render=True)

  def step(self, phis):
    self.tr.step(phis)
    self.pr.step(phis)

  def check(self):
    (tr_p, tr_v, _tr_u) = self.tr.getHeadcamPVU()
    (pr_p, pr_v, _pr_u) = self.pr.getHeadcamPVU()
    np.testing.assert_almost_equal(tr_p, pr_p, decimal=4)
    np.testing.assert_almost_equal(tr_v, pr_v, decimal=4)
    #np.testing.assert_almost_equal(tr_u, pr_u)

    #err_p = np.linalg.norm(tr_p - pr_p)
    #err_v = np.linalg.norm(tr_v - pr_v)
    #print("err_p=%f p=%s p=%s" % (err_p, tr_p, pr_p))
    #print("err_v=%f tr_v=%s pr_v=%s" % (err_v, tr_v, pr_v))

  def close(self):
    self.tr.close()
    self.pr.close()

    #pr._print_joints_pos()

NS, NP = 4, 4

if __name__ == "__main__":
    r = HybridRobot(NP, NP)

    while True:
      #phis = np.array([[np.pi/16, np.pi/16]] * 4, dtype=np.float32)
      #phis = np.array([[0,0],[0,0],[0,0],[0,0]], dtype=np.float32)
      phis = np.random.randn(NS, 2) - 0.5

      r.step(phis)
      r.check()

      r.pr.getCameraImage()