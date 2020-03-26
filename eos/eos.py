import numpy as np
from gym.envs.classic_control import rendering

class Env:
  S_LEN = 1

  """ This class implements world kinematics """
  def __init__(self, NJ):
    self.NJ = NJ
    self.TR = (2 + self.NJ) * self.S_LEN

    self.state = dict()

    self.set_target(0)
    self.set_phi(np.zeros(self.NJ))
    self.set_random_screw()

    self.viewer = None

  def set_random_screw(self):
    self.screw = np.random.uniform(size=(self.NJ,)) * 0.5 + 0.5 # [0.5 .. 1] range
    self.screw = self.screw * np.random.choice([-1., 1.]) # [ -1 .. -0.5, 0.5 .. -1] range

  def set_target(self, t_phi):
    self.t_coords = (self.TR * np.sin(t_phi), self.TR * np.cos(t_phi))

  def get_screw(self):
    return self.screw

  def set_phi(self, phi):
    """ Given phi does forward kinematics, updates joints world positions and angles, target view """
    
    self.phi = np.copy(phi)

    self.x = np.zeros(self.NJ,)
    self.y = np.zeros(self.NJ,)

    # calculate positions of the endpoints, each on top of the previous one
    for i in range(self.NJ):
      if i == 0:
        ep_phi = self.phi[i]
        self.x[i] = self.y[i] = 0
      else:
        ep_phi += self.phi[i]
        self.x[i] = self.x[i-1]
        self.y[i] = self.y[i-1]

      self.x[i] += self.S_LEN * np.sin(ep_phi)
      self.y[i] += self.S_LEN * np.cos(ep_phi)

    # calculate target view angle
    (t_dx, t_dy) = (self.t_coords[0] - self.x[-1], self.t_coords[1] - self.y[-1])
    ep_t_alpha = np.arctan2(t_dx, t_dy)
    self.alpha = ep_t_alpha - ep_phi
    self.ep_phi = ep_phi

  def get_sweep_phis(self):
    for phi in np.arange()
    return [[-1],[0],[1]]

  def render(self, mode='rgb_array'):
    # initialize viewer and draw axis
    bound1 = self.TR * 0.1
    bound2 = self.TR * 1.1
    if self.viewer is None:
      self.viewer = rendering.Viewer(100, 50)
      self.viewer.set_bounds(-bound2, bound2, -bound1, bound2)
    self.viewer.draw_line((-self.TR, 0), (self.TR, 0)).set_color(.5,.5, .5)
    self.viewer.draw_line((0, 0), (0, self.TR)).set_color(.5,.5, .5)

    x0 = y0 = 0
    for i in range(self.NJ):
      # draw blue lines for segments
      l = self.viewer.draw_line((x0, y0), (self.x[i], self.y[i]))
      l.set_color(0, 0, 1)

      # draw green circles for joints
      t = rendering.Transform(translation=(x0, y0))
      self.viewer.draw_circle(.025, color=(0, 1, 0)).add_attr(t)

      (x0, y0) = (self.x[i], self.y[i])

    # draw red circle for the target
    t = rendering.Transform(translation=self.t_coords)
    self.viewer.draw_circle(.05, color=(1, 0, 0)).add_attr(t)

    # draw alpha vector
    l = self.viewer.draw_line((self.x[-1], self.y[-1]),
      (self.x[-1] + self.S_LEN*np.sin(self.ep_phi + self.alpha),
      self.y[-1] + self.S_LEN*np.cos(self.ep_phi + self.alpha)))
    l.set_color(1, 0, 0)

    return self.viewer.render(return_rgb_array = mode=='rgb_array')

  def close(self):
    if self.viewer:
      self.viewer.close()
      self.viewer = None
