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

    r = HybridRobot(4, 4)

    #ls = np.array([[0,np.pi/8], [0,np.pi/4]], dtype=np.float32)
    #ls = np.array([[0, 0]] * NS, dtype=np.float32)

    #a0, a1, a2, a3 = np.random.rand(4) * np.pi/4
    #phis = np.array([[a0, 0], [a1, 0], [a2, 0], [a3, 0]], dtype=np.float32)
    #r.step(phis)

    p_head = np.array([0., 0., 0.8])
    r.pr.addHeadposMarker(p_head)

    while True:
        print("-" * 40)
        p_target = np.array([0.5 - np.random.rand(), 0., 1.5])
        z_target = p_target - p_head
        z_target = z_target / np.linalg.norm(z_target)
        r.pr.setTarget(p_target)

        for _ in range(3):
            r.tr.model.train_homing_v(p_head, z_target)
            phis = r.tr.model.get_phis()
            #print("phis=%s" % phis) #p.eval(session=self.sess))
            r.pr.step(phis)
            r.pr.getCameraImage()
            r.check()