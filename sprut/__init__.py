from gym.envs.registration import register

register(
    id='Sprut-v0',
    entry_point='sprut.envs:SprutEnv',
)

# env = gym.make('sprut:Sprut-v0')