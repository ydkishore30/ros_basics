#!/usr/bin/env python3
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

DOCK_ID = 'home_dock'


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for the docking action server directly instead of waitUntilNav2Active()
    # which does cross-container lifecycle service calls that can time out.
    navigator.info('Waiting for docking_server action...')
    navigator.docking_client.wait_for_server()
    navigator.info(f'Docking at: {DOCK_ID}')
    navigator.dockRobotByID(DOCK_ID)

    while not navigator.isTaskComplete():
        time.sleep(0.1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.info('Docking succeeded.')
    else:
        navigator.info(f'Docking failed with result: {result}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
