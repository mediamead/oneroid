import tensorflow as tf
import numpy as np
from model import TensorRobotModel

_JZ = 0.028
JZ = tf.constant(np.array([[_JZ], [_JZ], [_JZ]], dtype=np.float32))

class TensorRobot:
  def __init__(self, NS, NP, render=False):
    print("*** TensorRobot() initialized")
    self.NS = NS
    self.NP = NP
    self.model = TensorRobotModel(NS, NP)

  def step(self, phis):
    self.model.set(phis)

  def getHeadcamPVU(self):
    (p, x, _y, z) = self.model.get_pxyz()
    return p.flatten(), z.flatten(), x.flatten()
    
  def close(self):
      self.model.close()
      print("*** TensorRobot() closed")

  # def train(self):
  #   (p, v, u) = self.model_pvu_l
  #   p4 = tf.expand_dims(tf.constant([7, 0., 2], name="p4"), 1)
  #   #cost_v = tf.nn.l2_loss(v - v0)
  #   cost = tf.nn.l2_loss(p4 - p)
  #   #cost = cost_v + cost_p

  #   npc = np.array([[0.,0.], [0.,0.]], dtype=np.float32)
  #   vls = tf.Variable(npc) # Use variable 

  #   optimizer = tf.train.GradientDescentOptimizer(learning_rate = 0.001).minimize(cost)
  #   for _ in range(1):
  #       _ , c = self.sess.run([optimizer, cost], feed_dict={self.l: vls})

  #   writer = tf.summary.FileWriter('./graphs', self.sess.graph)
  #   writer.close()
  #   print("**********")

    # print("p4=%s" % p4.eval(session=sess))
    # print("p=%s" % p.eval(session=sess))
    # print("c=%s" % c)
    # print("l=%s" % l.eval(session=sess))
    # print("v=%s" % v.eval(session=sess))
    # print("u=%s" % u.eval(session=sess))
    
# #npc = np.array([[np.pi/8,0.],[-np.pi/8,0.],[-np.pi/8,0.],[np.pi/8,0.]], dtype=np.float32)
# a = 0 # np.sin(np.pi/4)
# npc = np.array([
#     [a, 0.],
#     [a, 0.],
#     [0.,0.],
#     [0.,0.]],
#     dtype=np.float32)
# l = tf.Variable(npc) # Use variable 

#l = tf.Variable(tf.zeros([NS, 2], tf.float32), name='l')

if __name__ == "__main__":
    r = TensorRobot(4, 4)
    ls = np.array([[0,0],[0,0],[0,0],[0,0]], dtype=np.float32)
    if True:
        r.step(ls)
        (p, v, u) = r.getHeadcamPVU()
        print("p=%s v=%s u=%s" % (p, v, u))
    r.close()