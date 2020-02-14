import tensorflow as tf
import numpy as np

_JZ = 0.028
JZ = tf.constant(np.array([[_JZ], [_JZ], [_JZ]], dtype=np.float32))

class TensorRobot:
  # String shifts are proportinal to sine of rotation angles they produce, if actuated independently.
  # If actuated together, they produce larger rotation angles.
  # Can build a graph, computing rotation matrix. Let sin(beta) = lx and sin(gamma) = ly.
  def mk_section(self, i, p, v, u, l):
    #i1 = i+1

    p_box = p + JZ * v

    # create Rxy rotation matrix
    sin_beta = l[0] # lx
    sin_gamma = l[1] # ly
    cos_beta = tf.math.sqrt(1. - sin_beta**2)
    cos_gamma = tf.math.sqrt(1. - sin_gamma**2)
    minus_sin_beta = tf.negative(sin_beta)
    minus_sin_gamma = tf.negative(sin_gamma)

    Ry = [[cos_beta, 0., sin_beta], [0., 1., 0.], [minus_sin_beta, 0, cos_beta]]
    v_box = tf.matmul(Ry, v)
    u_box = tf.matmul(Ry, u)

    Rx = [[1., 0., 0.], [0., cos_gamma, minus_sin_gamma], [0., sin_gamma, cos_gamma]]
    #Rx = [[1., 0., 0.], [0., cos_gamma, sin_gamma], [0., minus_sin_gamma, cos_gamma]]
    v_plate = tf.matmul(Rx, v_box) #, name=("Ryx%d" % i1))
    u_plate = tf.matmul(Rx, u_box) #, name=("Ryx%d" % i1))
    
    # rotate lower plate orientation vectors with Rxy to get orientation vectors of the upper plate
    #v1 = tf.matmul(Ryx, v) #, name=("v%d" % i1))
    #u1 = tf.matmul(Ryx, u) #, name=("u%d" % i1))
    
    # translate position of the lower plate with v and v1 to position of the upper plate
    #p1 = tf.add(p, JZ2 * tf.add(v, v1)) #, name=("p%d" % i1))
    p_plate = p_box + JZ * v_plate
    
    return (p_plate, v_plate, u_plate, [p_box, v_box, p_plate, v_plate])

  def mk_model(self, l):
        # base position
        p0 = tf.expand_dims(tf.constant([0., 0., 0.], name="p0"), 1)
        v0 = tf.expand_dims(tf.constant([0., 0., 1], name="v0"), 1)
        u0 = tf.expand_dims(tf.constant([-1., 0., 0.], name="u0"), 1)

        (p, v, u) = (p0, v0, u0)
        infos = []
        for i in range(self.NS):
          for j in range(self.NP):
            ll = l[i*self.NP+j]
            (p, v, u, info) = self.mk_section(i*self.NP+j, p, v, u, ll)
            #vs.append([tf.constant(i), tf.constant(j), ll])
            infos.append(info)
        return [p, v, u, infos]

  def __init__(self, NS, NP, render=False):
    print("*** Initializing TensorRobot(render=%s) ..." % render)

    self.NS = NS
    self.NP = NP
    self.sess = tf.compat.v1.Session()
    init = tf.compat.v1.global_variables_initializer() 
    self.sess.run(init)

    self.l = tf.placeholder("float", [self.NS*self.NP, 2])
    self.model_pvu_l = self.mk_model(self.l)
    #self.model_pvu_l[3].append(self.l)
    print("*** Initializing TensorRobot() done")

  def step(self, phis):
    self.lv = np.zeros((self.NS*self.NP, 2), dtype=np.float32)

    for i in range(self.NS):
      phix = phis[i][0] / self.NP
      phiy = phis[i][1] / self.NP
      sin_phix = np.sin(phix)
      sin_phiy = np.sin(phiy)

      for j in range(self.NP):
       k = i*self.NP+j
       self.lv[k][0] = sin_phix
       self.lv[k][1] = sin_phiy
       #print("## TENSORROBOT: section=%d plate=%d k=%d joint=%d phi=%.3f sin_phi=%.3f" %
       # (i, j, k, 2*k, phix, sin_phix))
       #print("## TENSORROBOT: section=%d plate=%d k=%d joint=%d phi=%.3f sin_phi=%.3f" %
       # (i, j, k, 2*k+1, phiy, sin_phiy))

  def getHeadcamPVU(self):
    #print("# lv.shape=%s lv=%s" % (self.lv.shape, self.lv))
    (p, v, u, infos) = self.sess.run(self.model_pvu_l, feed_dict={self.l: self.lv})
    print("#TR: p=%s v=%s u=%s" % (p, v, u))
    for (p_box, v_box, p_plate, v_plate) in infos:
      print("box   p=%s v=%s" % (p_box.flatten(), v_box.flatten()))
      print("plate p=%s v=%s" % (p_plate.flatten(), v_plate.flatten()))
    return (p, v, u, infos)

  def close(self):
      self.sess.close()
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