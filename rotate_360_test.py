import math, time, sys, rclpy
TURNS = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class T(Node):
    def __init__(s):
        super().__init__('rotate_360_test')
        s.pub = s.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        s.yaw = None; s.js = None
        s.create_subscription(Odometry, '/diff_drive_controller/odom', s.on_odom, 10)
        s.create_subscription(JointState, '/joint_states', s.on_js, 10)
    def on_odom(s, m):
        q = m.pose.pose.orientation
        s.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    def on_js(s, m): s.js = list(m.position)
    def send(s, wz):
        m = TwistStamped(); m.header.stamp = s.get_clock().now().to_msg(); m.twist.angular.z = wz
        s.pub.publish(m)

rclpy.init(); n = T()
t0 = time.time()
while (n.yaw is None or n.js is None) and time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
if n.yaw is None or n.js is None: print("no odom/joint_states"); raise SystemExit(1)
js0 = n.js[:]; last = n.yaw; total = 0.0
print(f"start yaw {math.degrees(last):.1f} deg, wheels {js0}")
try:
    t0 = time.time()
    next_turn = 1
    while abs(total) < TURNS*2*math.pi and time.time()-t0 < 40*TURNS:
        n.send(0.5); rclpy.spin_once(n, timeout_sec=0.05)
        d = (n.yaw - last + math.pi) % (2*math.pi) - math.pi; total += d; last = n.yaw
        if abs(total) >= next_turn*2*math.pi:
            print(f"turn {next_turn} done at {time.time()-t0:.1f}s", flush=True); next_turn += 1
finally:
    for _ in range(10): n.send(0.0); rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
d = (n.yaw - last + math.pi) % (2*math.pi) - math.pi; total += d
dl, dr = n.js[0]-js0[0], n.js[1]-js0[1]
print(f"time {time.time()-t0:.1f}s | odom turned {math.degrees(total):.1f} deg (incl. coast after stop)")
print(f"wheel rotation: left {dl:.2f} rad ({dl/(2*math.pi):.2f} rev), right {dr:.2f} rad ({dr/(2*math.pi):.2f} rev)")
print(f"expected for 360 deg x TURNS with sep 0.343, r 0.04: +-{TURNS*0.343/2*2*math.pi/0.04:.2f} rad each wheel (+-{TURNS*4.2875:.2f} rev)")
rclpy.shutdown()
