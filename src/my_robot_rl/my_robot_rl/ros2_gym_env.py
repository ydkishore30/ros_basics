#!/usr/bin/env python3
"""Gymnasium environment wrapping the ROS2/Gazebo differential-drive simulation."""
import math
import subprocess
import threading
import time
from typing import Optional, Tuple

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── Sensor / geometry constants ──────────────────────────────────────────────
NUM_LASER_BEAMS = 36       # downsampled from the full 360-beam scan
MAX_LASER_RANGE = 3.5      # metres — readings beyond this are clipped
COLLISION_DIST = 0.25      # metres — episode ends immediately on collision
GOAL_RADIUS = 0.10         # metres — episode ends when robot reaches this radius
MAX_STEPS = 500            # timesteps per episode before truncation

# ── Velocity limits ───────────────────────────────────────────────────────────
MAX_LINEAR_VEL = 0.5       # m/s
MAX_ANGULAR_VEL = 1.5      # rad/s

# ── Reward constants ──────────────────────────────────────────────────────────
REWARD_GOAL = 100.0
REWARD_COLLISION = -50.0
REWARD_STEP = -0.1         # small time penalty per step
PROGRESS_SCALE = 10.0      # scale factor for distance-reduction reward


class _RobotEnvNode(Node):
    """Internal ROS2 node: handles all publishers / subscribers for the env."""

    def __init__(self):
        super().__init__('robot_rl_env_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._lock = threading.Lock()
        self._laser: Optional[np.ndarray] = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._data_ready = threading.Event()

        self.create_subscription(LaserScan, '/scan', self._laser_cb, 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._odom_cb, 10
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _laser_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Readings below range_min are sensor artifacts (robot body reflections) — treat as free space
        invalid = np.isnan(ranges) | np.isinf(ranges) | (ranges < msg.range_min)
        ranges = np.where(invalid, MAX_LASER_RANGE, ranges)
        ranges = np.clip(ranges, 0.0, MAX_LASER_RANGE)
        step = max(1, len(ranges) // NUM_LASER_BEAMS)
        downsampled = ranges[::step][:NUM_LASER_BEAMS]
        # Pad to exactly NUM_LASER_BEAMS if the scan had fewer beams
        if len(downsampled) < NUM_LASER_BEAMS:
            downsampled = np.pad(
                downsampled, (0, NUM_LASER_BEAMS - len(downsampled)),
                constant_values=MAX_LASER_RANGE
            )
        with self._lock:
            self._laser = downsampled
        self._data_ready.set()

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self._lock:
            self._x = pos.x
            self._y = pos.y
            self._yaw = yaw

    # ── Accessors / commands ──────────────────────────────────────────────────

    def publish_velocity(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.publish_velocity(0.0, 0.0)

    def get_sensor_data(self) -> Tuple[Optional[np.ndarray], float, float, float]:
        with self._lock:
            laser = self._laser.copy() if self._laser is not None else None
            return laser, self._x, self._y, self._yaw


class RobotNavEnv(gym.Env):
    """
    Gymnasium environment for differential-drive robot navigation in Gazebo.

    Observation vector (NUM_LASER_BEAMS + 3,) — all values in [0, 1]:
        laser_0 … laser_{N-1}   normalised range readings
        dist_to_goal             normalised Euclidean distance to goal
        angle_to_goal            angle in [0,1] (mapped from [-π, π])
        heading_error            heading error in [0,1] (mapped from [-π, π])

    Action vector (2,) — continuous:
        linear_velocity  in [-MAX_LINEAR_VEL,  MAX_LINEAR_VEL]
        angular_velocity in [-MAX_ANGULAR_VEL, MAX_ANGULAR_VEL]

    Goals are cycled from the ``goal_positions`` list passed at construction.
    Between episodes the robot is teleported back to (0, 0, yaw=0) via the
    Gazebo ``/world/industrial-warehouse/set_pose`` service.
    """

    metadata = {'render_modes': ['human']}

    def __init__(self, goal_positions=None, render_mode=None):
        super().__init__()

        self._goals = goal_positions or [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (-1.0, 0.5),
        ]

        obs_dim = NUM_LASER_BEAMS + 3
        self.observation_space = gym.spaces.Box(
            low=np.zeros(obs_dim, dtype=np.float32),
            high=np.ones(obs_dim, dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = gym.spaces.Box(
            low=np.array([-MAX_LINEAR_VEL, -MAX_ANGULAR_VEL], dtype=np.float32),
            high=np.array([MAX_LINEAR_VEL,  MAX_ANGULAR_VEL], dtype=np.float32),
            dtype=np.float32,
        )

        if not rclpy.ok():
            rclpy.init()
        self._node = _RobotEnvNode()
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._ros_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._ros_thread.start()

        self._step_count = 0
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._prev_dist = 0.0
        self._goal_idx = 0
        # Odom origin captured at each reset so position is always relative to spawn
        self._odom_origin_x = 0.0
        self._odom_origin_y = 0.0

    # ── Gymnasium API ─────────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Stop firmly before teleport to flush any residual velocity
        for _ in range(5):
            self._node.stop()
            time.sleep(0.05)

        self._teleport_robot(0.0, 0.0, 0.0)
        time.sleep(1.2)  # let Gazebo physics settle after teleport

        # Stop again after teleport in case physics carried velocity
        for _ in range(3):
            self._node.stop()
            time.sleep(0.05)

        self._goal_x, self._goal_y = self._goals[self._goal_idx % len(self._goals)]
        self._goal_idx += 1
        self._step_count = 0

        self._node._data_ready.clear()
        self._node._data_ready.wait(timeout=5.0)

        # Capture odom at spawn so all distance calculations are spawn-relative.
        # The diff_drive_controller odom never resets between episodes, so we
        # zero it out here by remembering the offset.
        _, ox, oy, _ = self._node.get_sensor_data()
        self._odom_origin_x = ox
        self._odom_origin_y = oy

        obs = self._build_obs()
        self._prev_dist = self._dist_to_goal(ox, oy)
        return obs, {}

    def step(self, action: np.ndarray):
        linear = float(np.clip(action[0], -MAX_LINEAR_VEL, MAX_LINEAR_VEL))
        angular = float(np.clip(action[1], -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL))
        self._node.publish_velocity(linear, angular)

        time.sleep(0.05)  # 20 Hz control loop

        laser, x, y, yaw = self._node.get_sensor_data()
        self._step_count += 1

        collision = (laser is not None) and (np.min(laser) < COLLISION_DIST)
        dist = self._dist_to_goal(x, y)
        goal_reached = dist < GOAL_RADIUS
        truncated = self._step_count >= MAX_STEPS
        terminated = collision or goal_reached

        reward = REWARD_STEP + (self._prev_dist - dist) * PROGRESS_SCALE
        if goal_reached:
            reward += REWARD_GOAL
            self._node.get_logger().info(f'Goal reached at ({x:.2f},{y:.2f})')
        if collision:
            reward += REWARD_COLLISION
            self._node.stop()

        self._prev_dist = dist
        obs = self._build_obs()
        info = {'dist_to_goal': dist, 'step': self._step_count}
        return obs, reward, terminated, truncated, info

    def close(self):
        self._node.stop()
        self._executor.shutdown(wait=False)
        if rclpy.ok():
            rclpy.shutdown()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _build_obs(self) -> np.ndarray:
        laser, x, y, yaw = self._node.get_sensor_data()

        if laser is None:
            laser = np.full(NUM_LASER_BEAMS, MAX_LASER_RANGE, dtype=np.float32)

        laser_norm = laser / MAX_LASER_RANGE

        # Work in spawn-relative frame so odom drift doesn't corrupt goals
        rx = x - self._odom_origin_x
        ry = y - self._odom_origin_y

        dist = self._dist_to_goal(x, y)
        angle_to_goal = math.atan2(self._goal_y - ry, self._goal_x - rx)
        heading_err = self._wrap_angle(angle_to_goal - yaw)

        dist_norm = min(dist / 10.0, 1.0)
        angle_norm = (angle_to_goal + math.pi) / (2 * math.pi)
        heading_norm = (heading_err + math.pi) / (2 * math.pi)

        return np.concatenate([
            laser_norm.astype(np.float32),
            np.array([dist_norm, angle_norm, heading_norm], dtype=np.float32),
        ])

    def _dist_to_goal(self, x: float, y: float) -> float:
        rx = x - self._odom_origin_x
        ry = y - self._odom_origin_y
        return math.hypot(self._goal_x - rx, self._goal_y - ry)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _teleport_robot(self, x: float, y: float, yaw: float):
        """Reset robot pose in Gazebo via the set_pose service."""
        half_yaw = yaw / 2.0
        # gz service uses protobuf text format, not JSON
        req = (
            f'name: "my_robot" '
            f'position {{ x: {x} y: {y} z: 0.05 }} '
            f'orientation {{ x: 0 y: 0 '
            f'z: {math.sin(half_yaw):.6f} w: {math.cos(half_yaw):.6f} }}'
        )
        subprocess.run(
            [
                'gz', 'service', '-s', '/world/industrial-warehouse/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', req,
            ],
            capture_output=True,
        )
