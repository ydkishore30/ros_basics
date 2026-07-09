#!/usr/bin/env python3
"""
Train a PPO agent to navigate the differential-drive robot in Gazebo.

Usage (inside the container after colcon build):
    ros2 run my_robot_rl train_rl

Models are saved to ~/rl_models/ and TensorBoard logs to ~/rl_logs/.
Monitor training with:
    tensorboard --logdir ~/rl_logs
"""
import os

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from my_robot_rl.ros2_gym_env import RobotNavEnv

MODEL_DIR = os.path.expanduser('~/rl_models')
LOG_DIR = os.path.expanduser('~/rl_logs')

TOTAL_TIMESTEPS = 500_000

# Goals the robot must learn to reach (map frame, metres)
GOAL_POSITIONS = [
    (1.0,  0.0),
    (1.0,  1.0),
    (0.0,  1.0),
    (-1.0, 0.5),
]


def _make_env():
    env = RobotNavEnv(goal_positions=GOAL_POSITIONS)
    return Monitor(env, LOG_DIR)


def main():
    os.makedirs(MODEL_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

    env = DummyVecEnv([_make_env])

    final_model = os.path.join(MODEL_DIR, 'ppo_robot_nav_final.zip')
    if os.path.exists(final_model):
        print(f'Resuming from {final_model}')
        model = PPO.load(final_model, env=env, tensorboard_log=LOG_DIR,
                         custom_objects={'learning_rate': 1e-4, 'ent_coef': 0.05})
    else:
        model = PPO(
            policy='MlpPolicy',
            env=env,
            learning_rate=1e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.05,
            vf_coef=0.5,
            max_grad_norm=0.5,
            device='cpu',
            verbose=1,
            tensorboard_log=LOG_DIR,
        )

    checkpoint_cb = CheckpointCallback(
        save_freq=10_000,
        save_path=MODEL_DIR,
        name_prefix='ppo_robot_nav',
        verbose=1,
    )

    print(f'Starting PPO training for {TOTAL_TIMESTEPS:,} timesteps...')
    print(f'Models  → {MODEL_DIR}')
    print(f'Logs    → {LOG_DIR}  (tensorboard --logdir {LOG_DIR})')

    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        callback=checkpoint_cb,
    )

    final_path = os.path.join(MODEL_DIR, 'ppo_robot_nav_final')
    model.save(final_path)
    print(f'Training complete. Final model saved to {final_path}.zip')

    env.close()


if __name__ == '__main__':
    main()
