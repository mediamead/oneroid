import tensorflow as tf
import numpy as np

# Create a computational graph.
    
# 4 pairs of string shifts per section, lx and ly
NS = 4

class TensorRobot:
  # String shifts are proportinal to sine of rotation angles they produce, if actuated independently.
  # If actuated together, they produce larger rotation angles.
  # Can build a graph, computing rotation matrix. Let sin(beta) = lx and sin(gamma) = ly.
  def mk_section(self, i, p, v, u, l):
    i1 = i+1

    # create Rxy rotation matrix
    sin_beta = l[0] # lx
    sin_gamma = l[1] # ly
    cos_beta = tf.math.sqrt(1. - sin_beta**2)
    cos_gamma = tf.math.sqrt(1. - sin_gamma**2)
    minus_sin_beta = tf.negative(sin_beta)
    minus_sin_gamma = tf.negative(sin_gamma)
    Ry = [[cos_beta, 0., sin_beta], [0., 1., 0.], [minus_sin_beta, 0, cos_beta]]
    Rx = [[1., 0., 0.], [0, cos_gamma, minus_sin_gamma], [0., sin_gamma, cos_gamma]]
    Ryx = tf.matmul(Ry, Rx, name=("Ryx%d" % i1))
    
    # rotate lower plate orientation vectors with Rxy to get orientation vectors of the upper plate
    v1 = tf.matmul(Ryx, v, name=("v%d" % i1))
    u1 = tf.matmul(Ryx, u, name=("u%d" % i1))
    
    # translate position of the lower plate with v and v1 to position of the upper plate
    p1 = tf.add(p, tf.add(v, v1), name=("p%d" % i1))
    
    return (p1, v1, u1)

  def mk_model(self, l):
        # base position
        p0 = tf.expand_dims(tf.constant([0., 0., 0.], name="p0"), 1)
        v0 = tf.expand_dims(tf.constant([0., 0., 0.028], name="v0"), 1)
        u0 = tf.expand_dims(tf.constant([-1., 0., 0.], name="u0"), 1)

        (p, v, u) = (p0, v0, u0)
        for i in range(NS):
          for _ in range(4):
            (p, v, u) = self.mk_section(i, p, v, u, l[i,:] / 4)
        return [p, v, u]

  def __init__(self, render=False):
    print("*** Initializing TensorRobot(render=%s) ..." % render)

    self.sess = tf.compat.v1.Session()
    init = tf.compat.v1.global_variables_initializer() 
    self.sess.run(init)

    self.l = tf.placeholder("float", [NS, 2])
    self.model_pvu_l = self.mk_model(self.l)
    print("*** Initializing TensorRobot() done")

  def step(self, phis):
    self.ls = np.sin(phis / (np.pi/2))

  def getHeadcamPVU(self):
    (p, v, u) = self.sess.run(self.model_pvu_l, feed_dict={self.l: self.ls})
    return (p, v, u)

  def close(self):
      self.sess.close()
      print("*** TensorRobot() closed")

#   def train(self):
#     cost_v = 0 #tf.nn.l2_loss(v - v0)
#     cost_p = tf.nn.l2_loss(p4 - p)
#     cost = cost_v + cost_p

#     optimizer = tf.train.GradientDescentOptimizer(learning_rate = 0.001).minimize(cost)
#     for _ in range(1000):
#         _ , c = sess.run([optimizer, cost])

#     print("p4=%s" % p4.eval(session=sess))
#     print("p=%s" % p.eval(session=sess))
#     print("c=%s" % c)
#     print("l=%s" % l.eval(session=sess))
#     print("v=%s" % v.eval(session=sess))
#     print("u=%s" % u.eval(session=sess))
    
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
#p4 = tf.expand_dims(tf.constant([7, 0., 2], name="p4"), 1)

if __name__ == "__main__":
    r = TensorRobot()
    ls = np.array([[0,0],[0,0],[0,0],[0,0]], dtype=np.float32)
    if True:
        print("# ls=%s" % ls)
        r.step(ls)
        (p, v, u) = r.getHeadcamPVU()
        print("p=%s v=%s u=%s" % (p, v, u))
    r.close()