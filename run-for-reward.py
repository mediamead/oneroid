#!/usr/bin/env python

import gym
import copy

env = gym.make('eye_on_stick:EyeOnStick-v0')

while True:
  env.reset(keep_phi=True)
  for _ in range(500):
    env.render()
    action = env.action_space.sample()
    s0 = copy.deepcopy(env.state)
    print("%s %s" % (env.state["phi"], env.state["alpha"]))
    print(action)
    (_, reward, done, _) = env.step(action)
    print("%s %s" % (env.state["phi"], env.state["alpha"]))
    print("%f" % reward)
    if reward <= 0:
        env.state = s0
        print("-",)
    else:
        print("+",)
    if done: break

env.close()
