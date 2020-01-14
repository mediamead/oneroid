import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from numpy import cos, sin, arctan2

MIN_PHI = -0.1        # min/max joint rotation angle
MAX_PHI = 0.1
DPHI = np.pi /180 /2 # joint rotation angle delta per step: half degree

from sprut.envs.sprut_robot import Robot 

class SprutEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    self.robot = Robot()

    self.nphis = self.robot.get_nphis()

    # robot can rotate on any number of its joints on each step
    # each joint can have 3 rotation actions: CCW, no rotation, CW)
    self.action_space = spaces.MultiDiscrete(np.full((self.nphis,), 3))

    # 1x alpha + NJ x phi
    self.observation_space = spaces.Box(
      low=MIN_PHI, high=MAX_PHI, shape=(2 + self.nphis,), dtype=np.float32)

    self.seed()

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def get_obs(self):
    return np.concatenate([self.target, self.phis], axis=0)

  def reset(self):
    self.target = self.np_random.uniform(low=MIN_PHI, high=MAX_PHI, size=2) # FIXME
    self.robot.reset(self.target)
    self.phis = np.zeros(self.nphis)
    self.offCenter = self.robot.getOffCenter()
    return self.get_obs()

  def step(self, action):
    # update joint angles according to the effect of the action
    for i in range(self.nphis):
      a = action[i] - 1 # (0..2 => -1=CCW, 0=stay, 1=CW)
      dphi = DPHI * a
      self.phis[i] += dphi
      if self.phis[i] < MIN_PHI: self.phis[i] = MIN_PHI
      elif self.phis[i] > MAX_PHI: self.phis[i] = MAX_PHI

    self.robot.step(self.phis, nsteps=int(240/2))
    offCenter = self.robot.getOffCenter()
    reward, done = self.robot.getRewardDone(self.offCenter, offCenter)

    deltaOff = offCenter - self.offCenter
    log = "[%f %f %f] %f %s" % (self.offCenter, deltaOff, offCenter, reward, done)
    print(log)
    self.offCenter = offCenter

    return self.get_obs(), reward, done, {}
