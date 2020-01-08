#!/usr/bin/env python

import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

def train_and_measure(learn_steps):
    env = gym.make('eye_on_stick:EyeOnStick-v0')

    print("# Training for %d steps" % learn_steps)
    model = PPO2(MlpPolicy, env)
    model.learn(total_timesteps=learn_steps)
    print("# Training done")

    # measure average number of steps till success for 1000 epizodes
    N = 1000
    n = 0
    for _ in range(N):
        obs = env.reset()
        done = False
        for _ in range(250): # give model up to 250 steps to reach the goal
            action, _ = model.predict(obs)
            obs, _, done, _ = env.step(action)
            n = n + 1
            if done: break
    perf = n / N
    print("# Avg steps till done: %f" % (perf))
    return perf, model

#for learn_steps in [10000, 20000, 50000, 100000]:
#    train_and_measure(learn_steps)

best_perf = None
while True:
    perf, model = train_and_measure(10000)
    if best_perf is not None:
        if perf < best_perf:
            model.save("model")
            best_perf = perf
            print("found better performance %f" % perf)
    else:
        best_perf = perf
 