import math, time, sys, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

SIDE = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
V, W = 0.15, 0.5          # m/s, rad/s
LINE = len(sys.argv) > 2 and sys.argv[2] == 'line'   # one straight side, no turns

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return (a + math.pi) % (2*math.pi) - math.pi

class Sq(Node):
    def __init__(s):
        super().__init__('square_test')
        s.pub = s.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        s.odom = s.ekf = s.imu = None
        s.create_subscription(Imu, '/imu', lambda m: setattr(s, 'imu', m), 10)
        s.create_subscription(Odometry, '/diff_drive_controller/odom', lambda m: setattr(s, 'odom', m), 10)
        s.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(s, 'ekf', m), 10)
    def pose(s, m):
        p = m.pose.pose; return p.position.x, p.position.y, yaw(p.orientation)
    def send(s, v, w):
        m = TwistStamped(); m.header.stamp = s.get_clock().now().to_msg()
        m.twist.linear.x = v; m.twist.angular.z = w; s.pub.publish(m)
    def spin(s): rclpy.spin_once(s, timeout_sec=0.02)
    def stop(s, t=0.8):
        t0 = time.time()
        while time.time()-t0 < t: s.send(0.0, 0.0); s.spin()

rclpy.init(); n = Sq()
t0 = time.time()
while (n.odom is None or n.ekf is None or n.imu is None) and time.time()-t0 < 5: n.spin()
if n.odom is None or n.ekf is None: print("no odom / ekf"); raise SystemExit(1)

ekf0 = n.pose(n.ekf); od0 = n.pose(n.odom)
imu0 = yaw(n.imu.orientation) if n.imu else None
def rel(p0, p):   # pose p expressed in the start frame p0
    dx, dy = p[0]-p0[0], p[1]-p0[1]; c, s_ = math.cos(-p0[2]), math.sin(-p0[2])
    return dx*c - dy*s_, dx*s_ + dy*c, math.degrees(wrap(p[2]-p0[2]))
def report(tag):
    e = rel(ekf0, n.pose(n.ekf)); o = rel(od0, n.pose(n.odom))
    print(f"{tag:10s} RViz/EKF x {e[0]:6.3f} y {e[1]:6.3f} yaw {e[2]:7.1f} | wheel odom x {o[0]:6.3f} y {o[1]:6.3f} yaw {o[2]:7.1f}"
          + (f" | IMU yaw {math.degrees(wrap(yaw(n.imu.orientation)-imu0)):7.1f}" if imu0 is not None else ""), flush=True)

report("start")
try:
    for side in range(1 if LINE else 4):
        x0, y0, th0 = n.pose(n.odom)
        target_th = th0          # hold heading while driving straight
        t1 = time.time()
        while time.time()-t1 < SIDE/V*3:
            x, y, th = n.pose(n.odom); d = math.hypot(x-x0, y-y0); rem = SIDE - d
            if rem <= 0.005: break
            # Safety abort: robot can't hold heading or isn't moving (weak motor / low battery)
            if abs(wrap(target_th-th)) > math.radians(20) or (time.time()-t1 > 5 and d < 0.2):
                n.stop(); report("ABORT"); print(f"ABORTED on side {side+1}: heading error {math.degrees(wrap(target_th-th)):.1f} deg, moved {d:.2f} m in {time.time()-t1:.1f}s"); raise SystemExit(2)
            v = min(V, max(0.05, rem*1.0))
            n.send(v, max(-0.3, min(0.3, 2.0*wrap(target_th-th)))); n.spin()
        n.stop(); report(f"side {side+1}")
        if LINE: break
        t1 = time.time()
        while time.time()-t1 < 15:
            rem = wrap(th0 + math.pi/2 - n.pose(n.odom)[2])
            if rem <= math.radians(0.5): break
            n.send(0.0, min(W, max(0.15, rem*1.5))); n.spin()
        n.stop(); report(f"corner {side+1}")
finally:
    n.stop(1.0)
e = rel(ekf0, n.pose(n.ekf))
if not LINE: print(f"\nFINAL error vs start (RViz): {math.hypot(e[0], e[1])*100:.1f} cm, heading {e[2]:.1f} deg  (total time {time.time()-t0:.0f}s)")
rclpy.shutdown()
