import tensorflow as tf
import numpy as np
import quaternion

class TensorRobotModel(object):
  def __init__(self, NS, NP, JZ):
    self.NS = NS
    self.NP = NP
    self.JZ = JZ

    self.model = dict()
    self.model['phis'] = tf.Variable(tf.zeros([self.NS, 2]))

    pos_plate, pxyz_box = self.mk_pxyz_model(self.model['phis'])
    self.model['p'], self.model['x'], self.model['y'], self.model['z'] = pos_plate
    self.model['pxyz_box'] = pxyz_box

    self.model['p_target'] = tf.Variable([[0.]]*3, trainable=False)
    
    v_target = tf.nn.l2_normalize(self.model['p_target'] - self.model['p'])
    self.model['t_aim'] = tf.reduce_sum(tf.multiply(self.model['z'], v_target))

    self.sess = tf.compat.v1.Session()
    self.sess.run(tf.compat.v1.global_variables_initializer())

  def mk_pxyz_model_section(self, pos, phi):
    p, x, y, z = pos
    # translation from base plate to kardan
    p_box = p + self.JZ * z

    # create Rxy rotation matrix
    beta = phi[0]
    gamma = phi[1]

    # rotation matrix around y vector, for angle l[0]
    Ry = quaternion.R(y, beta)
    x_box = tf.matmul(Ry, x)
    y_box = tf.matmul(Ry, y)
    z_box = tf.matmul(Ry, z)

    # rotation matrix around x vector, for angle l[1]
    Rx = quaternion.R(x_box, -gamma)
    x_plate = tf.matmul(Rx, x_box)
    y_plate = tf.matmul(Rx, y_box)
    z_plate = tf.matmul(Rx, z_box)
    
    # translation from kardan to the next plate
    p_plate = p_box + self.JZ * z_plate
    
    return [p_plate, x_plate, y_plate, z_plate], [p_box, x_box, y_box, z_box]

  def mk_pxyz_model(self, l):
        # base position
        p0 = tf.expand_dims(tf.constant([0., 0., 0.], name="p0"), 1)
        x0 = tf.expand_dims(tf.constant([1., 0., 0.], name="x0"), 1)
        y0 = tf.expand_dims(tf.constant([0., 1., 0.], name="y0"), 1)
        z0 = tf.expand_dims(tf.constant([0., 0., 1.], name="z0"), 1)

        pos_plate = [p0, x0, y0, z0]

        for i in range(self.NS):
          for _ in range(self.NP):
            pos_plate, pxyz_box = self.mk_pxyz_model_section(pos_plate, l[i,:])

        return pos_plate, pxyz_box

  def set(self, phis):
    self.sess.run(self.model['phis'].assign(phis / self.NP))

  def get_pxyz(self):
    return self.sess.run([self.model['p'], self.model['x'], self.model['y'], self.model['z']])

  def close(self):
      self.sess.close()