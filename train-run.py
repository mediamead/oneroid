#!/usr/bin/env python

import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

#env = gym.make('CartPole-v1')
env = gym.make('eye_on_stick:EyeOnStick-v0')
# Optional: PPO2 requires a vectorized environment to run
# the env is now wrapped automatically when passing it to the constructor
# env = DummyVecEnv([lambda: env])

policy_kwargs = dict(net_arch=[1])
model = PPO2(MlpPolicy, env, learning_rate=0.001, verbose=1, policy_kwargs=policy_kwargs, tensorboard_log="./tensorboard/")
model.learn(total_timesteps=15000)

nsteps = 0
for _ in range(100):
    obs = env.reset() #keep_phi=True)
    done = False
    while not done:
        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        nsteps += 1
        env.render()

print(nsteps)