from gym.envs.registration import register

register(
    id='EyeOnStick-v0',
    entry_point='eye_on_stick.envs:EyeOnStickEnv',
)

# env = gym.make('eye_on_stick:EyeOnStick-v0')