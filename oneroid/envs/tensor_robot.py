import tensorflow as tf
import numpy as np
from model import TensorRobotModel

JZ = 0.028

class TensorRobot:
  def __init__(self, NS, NP):
    print("*** TensorRobot() initialized")
    self.NS = NS
    self.NP = NP
    self.model = TensorRobotModel(NS, NP, JZ)

  def step(self, phis):
    self.model.set(phis)

  def getHeadcamPVU(self):
    (p, x, _y, z) = self.model.get_pxyz()
    return p.flatten(), z.flatten(), x.flatten()
    
  def close(self):
      self.model.close()
      print("*** TensorRobot() closed")

if __name__ == "__main__":
    r = TensorRobot(4, 4)
    ls = np.array([[0,0],[0,0],[0,0],[0,0]], dtype=np.float32)
    if True:
        r.step(ls)
        (p, v, u) = r.getHeadcamPVU()
        print("p=%s v=%s u=%s" % (p, v, u))
    r.close()