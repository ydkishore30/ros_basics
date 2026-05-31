'''
Test EKF publishes odometry on /odometry/filtered.
'''

import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def test_ekf_publishing():
    '''Verify EKF is publishing odometry messages.'''

    rclpy.init()

    node = Node('ekf_test_node')
    msg_received = False

    def callback(msg):
        nonlocal msg_received
        msg_received = True

    # Subscribe to EKF output
    node.create_subscription(
        Odometry,
        '/odometry/filtered',
        callback,
        10
    )

    print('Waiting for EKF output on /odometry/filtered...')

    start = time.time()
    timeout = 10.0

    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if msg_received:
            print('EKF message received!')
            break

    node.destroy_node()
    rclpy.shutdown()

    assert msg_received, (
        'EKF NOT RUNNING or /odometry/filtered not publishing within 10 seconds'
    )