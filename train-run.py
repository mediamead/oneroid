#!/usr/bin/env python

import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

#env = gym.make('CartPole-v1')
env = gym.make('sprut:Sprut-v0', render=False)
# Optional: PPO2 requires a vectorized environment to run
# the env is now wrapped automatically when passing it to the constructor
# env = DummyVecEnv([lambda: env])

model = PPO2(MlpPolicy, env, verbose=1, tensorboard_log="./tensorboard/")
model.learn(total_timesteps=25000)
env.close()

env = gym.make('sprut:Sprut-v0', render=True)

while True:
    obs = env.reset() #keep_phi=True)
    done = False
    nsteps = 0
    while not done:
        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        nsteps += 1
        #env.render()
    print("nsteps %d" % nsteps)
