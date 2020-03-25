import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from numpy import cos, sin, arctan2

NJ = 1                # number of joints/sections
S_LEN = 1             # length of each section
MIN_PHI = -np.pi/3    # min/max joint rotation angle
MAX_PHI = np.pi/3
DPHI = np.pi / 180 * 3  # joint rotation angle delta per step: half degree
MIN_T_PHI = MIN_PHI # -np.pi/2  # min/max target angle
MAX_T_PHI = MAX_PHI # np.pi/2
TR = (2 + NJ) * S_LEN # distance to the target
ALPHA_DONE = np.pi / 180 * 10  # done when within +/- 2 degrees to the target

class EyeOnStickEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    self.viewer = None
    self.state = {}

    # robot can rotate on any number of its joints on each step
    # each joint can have 3 rotation actions: CCW, no rotation, CW)
    self.action_space = spaces.MultiDiscrete(np.full((NJ,), 3))

    # 1 x t_phi + 1x alpha + NJ x phi
    self.observation_space = spaces.Box(
      low=MIN_PHI, high=MAX_PHI,
      shape=(2*(NJ+1),),
      dtype=np.float32)

    self.seed()

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def reset(self, keep_phi=False):
    # randomly place target on a circle with radius of TR and random angle
    t_r = TR
    t_phi = self.np_random.uniform(low=MIN_T_PHI, high=MAX_T_PHI)
    t_coords = (t_r * sin(t_phi), t_r * cos(t_phi))

    # target: distance, angle, coords
    self.state["target"] = [t_r, t_phi, t_coords]

    # randomize joint angles, unless we have ones already and want to keep
    if (not "phi" in self.state) or not keep_phi:
      self.state["phi"] = np.zeros(NJ) # self.np_random.uniform(low=MIN_PHI, high=MAX_PHI, size=(NJ,))
      self.state["phi0"] = np.copy(self.state["phi"])

    if not "alpha" in self.state:
      self.state["alpha"] = 0.
      self.state["alpha0"] = self.state["alpha"]

    #self.screw = 0.75 + self.np_random.uniform(size=(NJ,)) * 0.25
    self.screw = self.np_random.choice([1.])

    #print("---")
    #print("t_phi: %.2f" % t_phi)
    #print("Initial phi[]: %s" % self.state["phi"])

    self.calc_state()
    return self._get_obs()

  def get_screw(self):
    return self.screw

  def _get_obs(self):
    obs = list()
    obs.append(self.state["phi"])
    obs.append(self.state["alpha"])
    obs.append(self.state["phi0"])
    obs.append(self.state["alpha0"])
    return obs

  def step(self, action):
    phi = self.state["phi"]
    self.state["phi0"] = np.copy(phi)
    done = False

    # update joint angles according to the effect of the action, accumulate costs
    #print("action=%s" % action)
    for i in range(NJ):
      a = action[i] - 1 # (0..2 => -1=CCW, 0=stay, 1=CW)
      dphi = DPHI * a
      #print("action: %s, dphi: %f" % (a, dphi))
      phi[i] += dphi
      if phi[i] < MIN_PHI: phi[i] = MIN_PHI
      elif phi[i] > MAX_PHI: phi[i] = MAX_PHI

    self.state["alpha0"] = self.state["alpha"]
    self.calc_state()
    alpha = self.state["alpha"]

    #d_alpha = np.abs(alpha0) - np.abs(alpha)
    #if d_alpha <=0 :
    #  reward = -10 # penalize moves in wrong direction
    #else:
    if np.abs(alpha) < ALPHA_DONE:
      reward = 10 # reward state aimed at target
      done = True
    else:
      reward = 0

    return self._get_obs(), reward, done, {}

  # calculate endpoint position and target view angle from the current robot pose (phis)
  def calc_state(self):
    phi = self.state["phi"] * self.screw
    x = self.state["_x"] = np.zeros(NJ,)
    y = self.state["_y"] = np.zeros(NJ,)

    # calculate positions of endpoints, each on top of previous one
    for i in range(NJ):
      if i == 0:
        ep_phi = phi[i]
        x[i] = y[i] = 0
      else:
        ep_phi += phi[i]
        x[i] = x[i-1]
        y[i] = y[i-1]

      x[i] += S_LEN * sin(ep_phi)
      y[i] += S_LEN * cos(ep_phi)

    # calculate target view angle
    t_coords = self.state["target"][2]
    (t_dx, t_dy) = (t_coords[0] - x[-1], t_coords[1] - y[-1])
    ep_t_alpha = arctan2(t_dx, t_dy)
    alpha = ep_t_alpha - ep_phi

    self.state["ep_phi"] = ep_phi
    self.state["alpha"] = alpha

  def render(self, mode='human'):
    from gym.envs.classic_control import rendering

    # initialize viewer and draw axis
    bound1 = TR * 0.1
    bound2 = TR * 1.1
    if self.viewer is None:
      self.viewer = rendering.Viewer(1000, 500)
      self.viewer.set_bounds(-bound2, bound2, -bound1, bound2)
    self.viewer.draw_line((-TR, 0), (TR, 0)).set_color(.5,.5, .5)
    self.viewer.draw_line((0, 0), (0, TR)).set_color(.5,.5, .5)

    s = self.state
    if s is None: return None
    x = self.state["_x"]
    y = self.state["_y"]
    t_coords = self.state["target"][2]

    x0 = y0 = 0
    for i in range(NJ):
      # draw blue lines for segments
      l = self.viewer.draw_line((x0, y0), (x[i], y[i]))
      l.set_color(0, 0, 1)

      # draw green circles for joints
      t = rendering.Transform(translation=(x0, y0))
      self.viewer.draw_circle(.025, color=(0, 1, 0)).add_attr(t)

      (x0, y0) = (x[i], y[i])

    # draw red circle for the target
    t = rendering.Transform(translation=t_coords)
    self.viewer.draw_circle(.05, color=(1, 0, 0)).add_attr(t)

    # draw alpha vector
    ep_phi = self.state["ep_phi"]
    alpha = self.state["alpha"]
    l = self.viewer.draw_line((x[-1], y[-1]),
      (x[-1] + S_LEN*sin(ep_phi + alpha), y[-1] + S_LEN*cos(ep_phi + alpha)))
    l.set_color(1, 0, 0)

    return self.viewer.render(return_rgb_array = mode=='rgb_array')

  def close(self):
    if self.viewer:
      self.viewer.close()
      self.viewer = None
