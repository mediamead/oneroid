import tensorflow as tf
import numpy as np

JZ = 0.028

class TensorRobotModel(object):
  def __init__(self, NS, NP):
    self.NS = NS
    self.NP = NP

    self.sess = tf.compat.v1.Session()
    init = tf.compat.v1.global_variables_initializer() 
    self.sess.run(init)

    self.l = tf.placeholder("float", [self.NS*self.NP, 2])
    self.model = self.mk_model(self.l)

  # String shifts are proportinal to sine of rotation angles they produce, if actuated independently.
  # If actuated together, they produce larger rotation angles.
  # Can build a graph, computing rotation matrix. Let sin(beta) = lx and sin(gamma) = ly.
  def mk_section(self, pos, l):
    p, x, y, z = pos
    # translation from base plate to kardan
    p_box = p + JZ * z

    # create Rxy rotation matrix
    sin_beta = l[0] # lx
    sin_gamma = l[1] # ly
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
    Rx = [[1., 0., 0.], [0., cos_gamma, minus_sin_gamma], [0., sin_gamma, cos_gamma]]
    x_plate = tf.matmul(Rx, x_box)
    y_plate = tf.matmul(Rx, y_box)
    z_plate = tf.matmul(Rx, z_box)
    
    # translation from kardan to the next plate
    p_plate = p_box + JZ * z_plate
    
    return ([p_plate, x_plate, y_plate, z_plate], [p_box, x_box, y_box, z_box])

  def mk_model(self, l):
        # base position
        p0 = tf.expand_dims(tf.constant([0., 0., 0.], name="p0"), 1)
        x0 = tf.expand_dims(tf.constant([1., 0., 1.], name="v0"), 1)
        y0 = tf.expand_dims(tf.constant([0., 1., 0.], name="u0"), 1)
        z0 = tf.expand_dims(tf.constant([0., 0., 1.], name="z0"), 1)

        pos_plate = [p0, x0, y0, z0]

        for i in range(self.NS):
          for _ in range(self.NP):
            pos_plate, _pos_box = self.mk_section(pos_plate, l[i,:])

        return pos_plate

  def set(self, phis):
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

  def run(self):
    #print("# lv.shape=%s lv=%s" % (self.lv.shape, self.lv))
    pos = self.sess.run(self.model, feed_dict={self.l: self.lv})
    #print("#TR: pos=%s" % pos)
    #for (p_box, v_box, p_plate, v_plate) in infos:
    #  print("box   p=%s v=%s" % (p_box.flatten(), v_box.flatten()))
    #  print("plate p=%s v=%s" % (p_plate.flatten(), v_plate.flatten()))
    return pos

    #p4 = tf.expand_dims(tf.constant([2., 0., 0.], name="p4"), 1)
  def close(self):
      self.sess.close()