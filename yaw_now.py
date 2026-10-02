import math, time, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
rclpy.init(); n = Node('yaw_now'); r = {}
n.create_subscription(Odometry, '/diff_drive_controller/odom', lambda m: r.__setitem__('wheel_odom', m), 10)
n.create_subscription(Odometry, '/odometry/filtered', lambda m: r.__setitem__('ekf', m), 10)
n.create_subscription(JointState, '/joint_states', lambda m: r.__setitem__('js', m), 10)
t0 = time.time()
while len(r) < 3 and time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
for k in ('wheel_odom', 'ekf'):
    if k in r:
        p = r[k].pose.pose; y = yaw(p.orientation)
        print(f"{k:11s}: yaw {y:7.2f} deg  (0-360: {y%360:6.2f})  x {p.position.x:.3f} m  y {p.position.y:.3f} m")
    else: print(f"{k}: no data")
if 'js' in r:
    L, R = r['js'].position
    print(f"wheels     : left {L:.2f} rad ({L/(2*math.pi):.2f} rev), right {R:.2f} rad ({R/(2*math.pi):.2f} rev)")
    print(f"yaw from wheel positions (r=0.04, sep=0.343): {math.degrees((R-L)*0.04/0.343):.2f} deg total since start")
rclpy.shutdown()
