import gym
env = gym.make('sprut:Sprut-v0')
while True:
  env.reset()
  info_shown = False
  for _ in range(1000):
    env.render()
    (obs, reward, done, info) = env.step(env.action_space.sample()) # take a random action
    if not info_shown:
        print(info)
        info_shown = True
env.close()
