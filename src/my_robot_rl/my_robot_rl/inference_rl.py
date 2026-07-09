#!/usr/bin/env python3
"""
Run a trained PPO model on the live Gazebo simulation.

Usage (inside container after colcon build):
    ros2 run my_robot_rl inference_rl

Expects a saved model at ~/rl_models/ppo_robot_nav_final.zip
(produced by train_rl).
"""
import os
import time

from stable_baselines3 import PPO

from my_robot_rl.ros2_gym_env import RobotNavEnv

MODEL_PATH = os.path.expanduser('~/rl_models/ppo_robot_nav_final')

GOAL_POSITIONS = [
    (1.0,  0.0),
    (1.0,  1.0),
    (0.0,  1.0),
    (-1.0, 0.5),
]

NUM_EPISODES = 5


def main():
    model_file = MODEL_PATH + '.zip'
    if not os.path.exists(model_file):
        print(f'[inference_rl] No model found at {model_file}')
        print('Run `ros2 run my_robot_rl train_rl` first.')
        return

    env = RobotNavEnv(goal_positions=GOAL_POSITIONS)
    model = PPO.load(MODEL_PATH, env=env)
    print(f'[inference_rl] Loaded model from {MODEL_PATH}')

    for ep in range(NUM_EPISODES):
        obs, _ = env.reset()
        total_reward = 0.0
        done = False

        print(f'\n── Episode {ep + 1}/{NUM_EPISODES} ──')
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            done = terminated or truncated
            print(
                f'  step {info["step"]:3d} | '
                f'dist={info["dist_to_goal"]:.3f} m | '
                f'reward={reward:+.2f}'
            )

        print(f'Episode {ep + 1} finished — total reward: {total_reward:.1f}')

    env.close()
    print('\nInference complete.')


if __name__ == '__main__':
    main()
