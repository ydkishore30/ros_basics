#!/usr/bin/env python3
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

INITIAL_X = 0.0
INITIAL_Y = 0.0
INITIAL_YAW = 0.0  # radians

WAYPOINTS = [
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 1.57),
    (0.0, 1.0, 3.14),
    (0.0, 0.0, -1.57),
]


def make_pose(navigator, x, y, yaw):
    q = quaternion_from_euler(0.0, 0.0, yaw)

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def set_initial_pose(navigator, x, y, yaw):
    navigator.setInitialPose(make_pose(navigator, x, y, yaw))


def create_waypoints(navigator):
    return [make_pose(navigator, x, y, yaw) for x, y, yaw in WAYPOINTS]


def main():
    rclpy.init()
    navigator = BasicNavigator()

    set_initial_pose(navigator, INITIAL_X, INITIAL_Y, INITIAL_YAW)
    navigator.waitUntilNav2Active()
    navigator.get_logger().info('Initial pose set and Nav2 is active.')

    waypoints = create_waypoints(navigator)
    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        time.sleep(0.1)

    navigator.get_logger().info('Finished following waypoints.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
