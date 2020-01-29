import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from numpy import cos, sin, arctan2

MAX_PHI = 0.25         # min/max joint rotation angle
MAX_TARGET_XY = 0.25
TARGET_Z = 1

from sprut.robots.sprut import Robot 

class SprutEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, render=False):
    self.robot = Robot(render)

    self.nphis = self.robot.get_nphis()

    # phi
    self.action_space = spaces.Box(
      low=-1, high=1,
      shape=(self.nphis,), dtype=np.float32)

    # phis + dx & dy
    self.observation_space = spaces.Box(
      low=-1, high=1, shape=(1 + 2,), dtype=np.float32)

    self.seed()

  def close(self):
    self.robot.close()
    
  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def reset(self):
    self.target = self.np_random.uniform(low=-MAX_TARGET_XY, high=MAX_TARGET_XY, size=2)
    self.target[1] = 0 # zero out Y
    self.robot.setTarget([self.target[0], self.target[1], TARGET_Z])
    self.phis = np.zeros(self.nphis)

    self.robot.update()
    return self.get_obs()

  def step(self, action):
    for i in range(self.nphis):
      self.phis[i] = action[i] * MAX_PHI

    self.robot.step([self.phis[0], 0, 0, 0, 0, 0, 0, 0])
    self.robot.update()

    return self.get_obs(), self.robot.qval, self.robot.done, {}
  
  def get_obs(self):
    return [self.phis[0]/MAX_PHI, self.robot.dx, self.robot.dy]

