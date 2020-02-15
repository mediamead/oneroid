import tensorflow as tf
import numpy as np

JZ = 0.028

class TensorRobotModel(object):
  def __init__(self, NS, NP):
    self.NS = NS
    self.NP = NP

    self.model = dict()
    self.model['phis'] = tf.Variable(tf.zeros([self.NS, 2]))

    # Y is boka! don't train! FIXME
    for i in range(self.NS):
        self.model['phis'][i, 1].is_trainable = False
    
    self.model['p'], self.model['x'], self.model['y'], self.model['z'] = \
      self.mk_pxyz_model(self.model['phis'])

    self.sess = tf.compat.v1.Session()
    init = tf.compat.v1.global_variables_initializer() 
    self.sess.run(init)

  # String shifts are proportinal to sine of rotation angles they produce, if actuated independently.
  # If actuated together, they produce larger rotation angles.
  # Can build a graph, computing rotation matrix. Let sin(beta) = lx and sin(gamma) = ly.
  def mk_pxyz_model_section(self, pos, phi):
    p, x, y, z = pos
    # translation from base plate to kardan
    p_box = p + JZ * z

    # create Rxy rotation matrix
    sin_beta = phi[0]
    sin_gamma = phi[1]
    cos_beta = tf.math.sqrt(1. - sin_beta**2)
    cos_gamma = tf.math.sqrt(1. - sin_gamma**2)
    minus_sin_beta = tf.negative(sin_beta)
    minus_sin_gamma = tf.negative(sin_gamma)

    # rotation matrix around y vector, for angle l[0]
    Ry = [[cos_beta, 0., sin_beta], [0., 1., 0.], [minus_sin_beta, 0, cos_beta]]

    x_box = tf.matmul(Ry, x)
    y_box = tf.matmul(Ry, y)
    z_box = tf.matmul(Ry, z)

    # rotation matrix around x vector, for angle l[1]
    Rx = np.identity(3, dtype=np.float32) # [[1., 0., 0.], [0., cos_gamma, minus_sin_gamma], [0., sin_gamma, cos_gamma]]
    x_plate = tf.matmul(Rx, x_box)
    y_plate = tf.matmul(Rx, y_box)
    z_plate = tf.matmul(Rx, z_box)
    
    # translation from kardan to the next plate
    p_plate = p_box + JZ * z_plate
    
    return [p_plate, x_plate, y_plate, z_plate]

  def mk_pxyz_model(self, l):
        # base position
        p0 = tf.expand_dims(tf.constant([0., 0., 0.], name="p0"), 1)
        x0 = tf.expand_dims(tf.constant([1., 0., 1.], name="x0"), 1)
        y0 = tf.expand_dims(tf.constant([0., 1., 0.], name="y0"), 1)
        z0 = tf.expand_dims(tf.constant([0., 0., 1.], name="z0"), 1)

        pos_plate = [p0, x0, y0, z0]

        for i in range(self.NS):
          for _ in range(self.NP):
            pos_plate = self.mk_pxyz_model_section(pos_plate, l[i,:])

        return pos_plate

  def set(self, phis):
    lv = np.zeros((self.NS, 2), dtype=np.float32)

    for i in range(self.NS):
      phix = phis[i][0] / self.NP
      phiy = phis[i][1] / self.NP

      sin_phix = np.sin(phix)
      sin_phiy = np.sin(phiy)

      lv[i][0] = sin_phix
      lv[i][1] = sin_phiy

    self.model['phis'].assign(lv)

  def get(self):
    phis = np.zeros((self.NS, 2), dtype=np.float32)
    lv = self.model['phis'].eval(session=self.sess)

    for i in range(self.NS):
      sin_phix = lv[i][0]
      sin_phiy = lv[i][1]

      phix = np.arcsin(sin_phix) * self.NP
      phiy = np.arcsin(sin_phiy) * self.NP

      phis[i][0] = phix
      phis[i][1] = phiy

    return phis

  #def run(self):
    #print("# lv.shape=%s lv=%s" % (self.lv.shape, self.lv))
    #res = self.sess.run(self.model)
    #print("#TR: pos=%s" % pos)
    #for (p_box, v_box, p_plate, v_plate) in infos:
    #  print("box   p=%s v=%s" % (p_box.flatten(), v_box.flatten()))
    #  print("plate p=%s v=%s" % (p_plate.flatten(), v_plate.flatten()))
    #return res

  head_orn = tf.expand_dims(tf.constant([0., 0., 1.]), 1)

  def get_pxyz(self):
      return self.sess.run([self.model['p'], self.model['x'], self.model['y'], self.model['z']])

  def train_homing_v(self, p_target):
    p_target = tf.expand_dims(tf.constant(p_target), 1)

    cost_v = tf.nn.l2_loss(self.model['z'] - self.head_orn) # v_plate
    cost_p = tf.nn.l2_loss(self.model['p'] - p_target) # distance between p_plate and the target
    cost = cost_v + cost_p/10

    optimizer = tf.train.GradientDescentOptimizer(learning_rate = 0.2).minimize(cost)
    for _ in range(1000):
        _ , c, p, z, lv = self.sess.run(
          [optimizer, [cost, cost_p, cost_v], self.model['p'], self.model['z'], self.model['phis']])
        #print(c)

    print("c=%s" % c)
    print("l=%s" % lv)
    print("p=%s, z=%s" % (p, z))

  def close(self):
      self.sess.close()