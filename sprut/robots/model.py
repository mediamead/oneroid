import tensorflow as tf
import numpy as np
import quaternion

JZ = 0.028

class TensorRobotModel(object):
  def __init__(self, NS, NP):
    self.NS = NS
    self.NP = NP

    self.model = dict()
    self.model['phis'] = tf.Variable(tf.zeros([self.NS, 2]))
    for i in range(self.NS): self.model['phis'][i, 0].is_trainable = False # FIXME

    pos_plate, pxyz_box = self.mk_pxyz_model(self.model['phis'])
    self.model['p'], self.model['x'], self.model['y'], self.model['z'] = pos_plate
    self.model['pxyz_box'] = pxyz_box

    self.model['p_target'] = tf.Variable([[0.]]*3, trainable=False)
    self.model['x_target'] = tf.Variable([[0.]]*3, trainable=False)
    self.model['y_target'] = tf.Variable([[0.]]*3, trainable=False)
    self.model['z_target'] = tf.Variable([[0.]]*3, trainable=False)

    # optimize position and orientaton of z-axis
    self.model['cost1_p'] = tf.nn.l2_loss(self.model['p'] - self.model['p_target'])
    self.model['cost1_x'] = tf.nn.l2_loss(self.model['x'] - self.model['x_target'])
    self.model['cost1_y'] = tf.nn.l2_loss(self.model['y'] - self.model['y_target'])
    self.model['cost1_z'] = tf.nn.l2_loss(self.model['z'] - self.model['z_target'])
    self.model['cost1'] = self.model['cost1_y'] + self.model['cost1_z'] + self.model['cost1_p'] # no X axis homing
    self.model['opt1'] = tf.train.GradientDescentOptimizer(learning_rate = 0.025).minimize(self.model['cost1'])

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
    p_plate = p_box + JZ * z_plate
    
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
    #phis = tf.expand_dims(phis, 0)
    self.sess.run(self.model['phis'].assign(phis / self.NP))

  def get_phis(self):
    return self.model['phis'].eval(session=self.sess) * self.NP

  def get_pxyz(self):
    return self.sess.run([self.model['p'], self.model['x'], self.model['y'], self.model['z']])
    #res = self.sess.run([self.model['p'], self.model['x'], self.model['y'], self.model['z'], self.model['pxyz_box']])
    #print("get_pxyz=%s" % res)
    #return res[0], res[1], res[2], res[3]

  def homing_pxyz(self, p_head, x_head, y_head, z_head):
    p_head = np.expand_dims(np.array(p_head).transpose(), axis=1)
    x_head = np.expand_dims(np.array(x_head).transpose(), axis=1) / np.linalg.norm(x_head)
    y_head = np.expand_dims(np.array(y_head).transpose(), axis=1) / np.linalg.norm(y_head)
    z_head = np.expand_dims(np.array(z_head).transpose(), axis=1) / np.linalg.norm(z_head)
    self.sess.run([
      self.model['p_target'].assign(p_head),
      self.model['x_target'].assign(x_head),
      self.model['y_target'].assign(y_head),
      self.model['z_target'].assign(z_head)
    ])

    for _ in range(10):
        #_ , c, pxyz, phis, dbg = self.sess.run(
        _ , c = self.sess.run(
          [
            self.model['opt1'],
            [self.model['cost1'], self.model['cost1_p'], self.model['cost1_x'], self.model['cost1_y'], self.model['cost1_z']],
            #[self.model['p'], self.model['x'], self.model['y'], self.model['z']], self.model['phis'],
            #[self.model['pxyz_box']]
          ])

    print("c=%s" % c)
    #print("l=%s" % lv)
    #print("p=%s, z=%s" % (pxyz[0], pxyz[3]))
    #print("dbg=%s" % dbg)

    #return pxyz, phis

  def close(self):
      self.sess.close()