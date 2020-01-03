import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from numpy import cos, sin, arctan2

NJ = 3                # number of joints/sections
S_LEN = 1             # length of each section
MIN_PHI = -np.pi/3    # min/max joint rotation angle
MAX_PHI = np.pi/3
DPHI = np.pi / 180    # joint rotation angle delta per step
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
      "phi": spaces.Box(low=MIN_PHI, high=MAX_PHI, shape=(NJ,), dtype=np.float32), # joint angles
    })

    self.sections = np.full(NJ, S_LEN)

    self.np_random, _  = seeding.np_random()

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

      # calculate positions of the end of the section, on top of previous one
      if i == 0:
        abs_phi = 0
        x[i] = y[i] = 0
      else:
        x[i] = x[i-1]
        y[i] = y[i-1]

      abs_phi += phi[i]
      x[i] += S_LEN * sin(abs_phi)
      y[i] += S_LEN * cos(abs_phi)

      # accumulate cost of joint movements
      #   lower joints are more expensive to move
      cost += action[i]**2 * (NJ-i)**2 # 0 .. 9
      #   stronger bends are more expensive 
      cost += (phi[i])**4 # 0 .. 1.2 (assuming 60 degrees max bends)

    reward, done = self._get_reward_done(x[-1], y[-1], abs_phi) - cost

    return self._get_obs(), reward, done, {}

  def reset(self):
    t_phi = self.np_random.uniform(low=MIN_T_PHI, high=MAX_T_PHI)
    t_x = TR * cos(t_phi)
    t_y = TR * sin(t_phi)

    self.state = {
      # joint angles
      "phi": self.np_random.uniform(low=MIN_PHI, high=MAX_PHI, size=(NJ,)),
      # target angle
      "target": [t_x, t_y],
      "_x": np.zeros(NJ,),
      "_y": np.zeros(NJ,)
    }
    return self._get_obs()

  def _get_obs(self):
    phi = self.state["phi"]
    return np.array([
      cos(phi[0]), sin(phi[0]),
      cos(phi[1]), sin(phi[1]),
      cos(phi[2]), sin(phi[2]), # angles FIXME

    ])

  def _get_reward_done(self, x, y, phi):
    t_x = self.state["target"][0] - x
    t_y = self.state["target"][1] - y
    # d = sqrt(dx**2 + dy**2)
    alpha = np.abs(phi - arctan2(t_x, t_y))
    reward = (np.pi/2 - alpha) / (np.pi/2) * 90 # 0 .. 90
    done = (alpha < ALPHA_DONE)
    return reward, done

  def render(self, mode='human'):
    from gym.envs.classic_control import rendering

    if self.viewer is None:
      self.viewer = rendering.Viewer(500, 500)
      bound1 = TR * 0.1
      bound2 = TR * 1.1
      self.viewer.set_bounds(-bound2, bound2, -bound1, bound2)

    self.viewer.draw_line((-TR, 0), (TR, 0))
    self.viewer.draw_line((0, 0), (0, TR))

    s = self.state
    if s is None: return None
    x = self.state["_x"]
    y = self.state["_y"]

    x0 = y0 = 0
    for i in range(NJ):
      self.viewer.draw_line((x0, y0), (x[i], y[i]))
      x0 = x[i]
      y0 = y[i]

    return self.viewer.render(return_rgb_array = mode=='rgb_array')

  def close(self):
    if self.viewer:
      self.viewer.close()
      self.viewer = None
