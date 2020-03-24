#!/usr/bin/env python

import gym

from stable_baselines.common.policies import MlpPolicy, MlpLnLstmPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2
from stable_baselines.common.vec_env import SubprocVecEnv

if __name__ == '__main__':
    n_cpu = 1
    env = SubprocVecEnv([lambda: gym.make('eye_on_stick:EyeOnStick-v0') for i in range(n_cpu)])

    model = PPO2(MlpLnLstmPolicy, env, nminibatches=1, verbose=1, tensorboard_log="./tensorboard/")
    model.learn(total_timesteps=100000)

    while True:
        obs = env.env_method('reset') #keep_phi=True)
        print(env.env_method('get_screw', indices=[0]))
        done = [False]
        while not all(done):
            action, _states = model.predict(obs)
            obs, rewards, done, info = env.step(action)
            env.env_method('render', indices=[0])
