from gym.envs.registration import register

register(
    id='Oneroid-v0',
    entry_point='oneroid.envs:OneroidEnv'
)

# env = gym.make('eye_on_stick:EyeOnStick-v0')