import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from numpy import cos, sin, arctan2

NJ = 3                # number of joints/sections
S_LEN = 1             # length of each section
MIN_PHI = -np.pi/3    # min/max joint rotation angle
MAX_PHI = np.pi/3
DPHI = np.pi /180/2   # joint rotation angle delta per step: half degree
MIN_T_PHI = -np.pi/2  # min/max target angle
MAX_T_PHI = np.pi/2
TR = (NJ + 1) * S_LEN # distance to the target
ALPHA_DONE = np.pi / 180  # done when within 1 degree to the target

class EyeOnStickEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    self.viewer = None
    self.state = None

    # robot can rotate on any number of its joints on each step
    # each joint rotation action can be: -1 (CCW), 0 (no rotation), or 1(CW)
    self.action_space = spaces.Box(low=-1, high=1, shape=(NJ,))

    self.observation_space = spaces.Dict({
      "t_phi": spaces.Box(low=MIN_T_PHI, high=MAX_T_PHI, shape=(1, )),
      "phi": spaces.Box(low=MIN_PHI, high=MAX_PHI, shape=(NJ,), dtype=np.float32), # joint angles
    })

    self.sections = np.full(NJ, S_LEN)

    self.np_random, _  = seeding.np_random()

  def reset(self):
    # place target on a circle with radius of TR and random angle
    t_r = TR
    t_phi = self.np_random.uniform(low=MIN_T_PHI, high=MAX_T_PHI)
    t_coords = (t_r * sin(t_phi), t_r * cos(t_phi))

    self.state = {
      # target: distance, angle, coords
      "target": [t_r, t_phi, t_coords],
      # joints: angles
      "phi": self.np_random.uniform(low=MIN_PHI, high=MAX_PHI, size=(NJ,)),
      # allocate space for coords of section joints
      "_x": np.zeros(NJ,),
      "_y": np.zeros(NJ,)
    }

    print("---")
    print("t_phi: %.2f" % t_phi)
    print("Initial phi[]: %s" % self.state["phi"])

    return self._get_obs()

  def _get_obs(self):
    t_phi = self.state["target"][1]
    phi = self.state["phi"]
    return np.array([
      t_phi, phi[0], phi[1], phi[2] # angles FIXME
    ])

  def step(self, action):
    phi = self.state["phi"]
    x = self.state["_x"]
    y = self.state["_y"]
    cost = 0

    # update joint angles, section endpoint coordinates, accumulate costs
    for i in range(NJ):
      dphi = DPHI * action[i]
      phi[i] += dphi
      if phi[i] < MIN_PHI: phi[i] = MIN_PHI
      elif phi[i] > MAX_PHI: phi[i] = MAX_PHI

      # calculate positions of endpoints, each on top of previous one
      if i == 0:
        ep_phi = 0
        x[i] = y[i] = 0
      else:
        x[i] = x[i-1]
        y[i] = y[i-1]

      ep_phi += phi[i]
      x[i] += S_LEN * sin(ep_phi)
      y[i] += S_LEN * cos(ep_phi)

      # accumulate cost of joint movements
      #   lower joints are more expensive to move
      cost += action[i]**2 * (NJ-i)**2 # 0 .. 9
      #   stronger bends are more expensive 
      cost += (phi[i])**4 # 0 .. 1.2 (assuming 60 degrees max bends)

    reward, done, info = self._get_reward_done(x[-1], y[-1], ep_phi)
    return self._get_obs(), reward - cost, done, info

  def _get_reward_done(self, x, y, ep_phi):
    # calc absolute view angle to the target
    t_coords = self.state["target"][2]
    (t_dx, t_dy) = (t_coords[0] - x, t_coords[1] - y)

    ep_t_alpha = arctan2(t_dx, t_dy)
    alpha = np.abs(ep_phi - ep_t_alpha)

    d = np.sqrt(t_dx**2 + t_dy**2)
    # max_d = TR + S_LEN * NJ => 4*S_LEN
    # min_d = TR - S_LEN * NJ = S_LEN
    d_reward = (2 - d / S_LEN) * 10 # 20 .. -20

    # calculate rewards
    a_reward = 100 - alpha / (np.pi/2) * 90 # 100 .. -80
    reward = a_reward + d_reward
    done = (alpha < ALPHA_DONE)

    #info = "ep_t_alpha: %6.2f, ep_phi: %6.2f, reward: %6.2f" % \
    #  (ep_t_alpha, ep_phi, reward)
    info = "alpha: %6.2f, d: %6.2f => a_reward: %6.2f, d_reward: %6.2f, reward: %6.2f" % \
      (alpha, d, a_reward, d_reward, reward)

    return reward, done, info

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

    return self.viewer.render(return_rgb_array = mode=='rgb_array')

  def close(self):
    if self.viewer:
      self.viewer.close()
      self.viewer = None
