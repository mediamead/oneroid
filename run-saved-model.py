#!/usr/bin/env python

import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

env = gym.make('eye_on_stick:EyeOnStick-v0')
model = PPO2.load("model")

while True:
  obs = env.reset(keep_phi=True)

  for _ in range(500):
    env.render()

    action, _ = model.predict(obs)
    obs, _, done, _ = env.step(action)

    if done: break

env.close()
