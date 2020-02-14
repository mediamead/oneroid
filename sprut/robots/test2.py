#!/usr/bin/env python3

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

    r = HybridRobot(4, 1)

    #ls = np.array([[0,np.pi/8], [0,np.pi/4]], dtype=np.float32)
    #ls = np.array([[0, 0]] * NS, dtype=np.float32)

    #a0, a1, a2, a3 = np.random.rand(4) * np.pi/4
    #phis = np.array([[a0, 0], [a1, 0], [a2, 0], [a3, 0]], dtype=np.float32)
    #r.step(phis)

    while True:
        p4 = [0.5 - np.random.rand(), 0., 0.5]
        r.pr.setTarget(p4)

        for _ in range(5):
            r.tr.model.train(p4)
            phis = r.tr.model.get()
            print("phis=%s" % phis) #p.eval(session=self.sess))
            r.pr.step(phis)
            r.pr.getCameraImage()
            r.check()