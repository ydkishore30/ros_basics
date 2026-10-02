# go_to.py FWD LEFT [yaw_deg]: drive to a point FWD m ahead and LEFT m to the left of
# the robot's current pose (RViz/EKF frame). Optional yaw_deg: final heading relative
# to the start heading. Reverses if the target is behind, otherwise turns and drives.
import math, time, sys, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

V, W = 0.15, 0.5
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return (a + math.pi) % (2*math.pi) - math.pi

class Home(Node):
    def __init__(s):
        super().__init__('go_to')
        s.pub = s.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        s.ekf = None
        s.create_subscription(Odometry, '/odometry/filtered', lambda m: setattr(s, 'ekf', m), 10)
    def pose(s):
        p = s.ekf.pose.pose; return p.position.x, p.position.y, yaw(p.orientation)
    def send(s, v, w):
        m = TwistStamped(); m.header.stamp = s.get_clock().now().to_msg()
        m.twist.linear.x = v; m.twist.angular.z = w; s.pub.publish(m)
    def spin(s): rclpy.spin_once(s, timeout_sec=0.02)
    def stop(s, t=0.8):
        t0 = time.time()
        while time.time()-t0 < t: s.send(0.0, 0.0); s.spin()
    def turn_to(s, target):
        t0 = time.time()
        while time.time()-t0 < 20:
            e = wrap(target - s.pose()[2])
            if abs(e) < math.radians(1.0): break
            s.send(0.0, math.copysign(min(W, max(0.15, abs(e)*1.5)), e)); s.spin()
        s.stop()
    def report(s, tag):
        x, y, th = s.pose(); print(f"{tag:8s} x {x:6.3f} y {y:6.3f} yaw {math.degrees(th):6.1f}  (dist to goal {math.hypot(x-GX, y-GY)*100:.1f} cm)", flush=True)

rclpy.init(); n = Home()
t0 = time.time()
while n.ekf is None and time.time()-t0 < 5: n.spin()
if n.ekf is None: print("no /odometry/filtered"); raise SystemExit(1)
sx, sy, sth = n.pose()
FWD, LEFT = float(sys.argv[1]), float(sys.argv[2])
GX = sx + FWD*math.cos(sth) - LEFT*math.sin(sth)
GY = sy + FWD*math.sin(sth) + LEFT*math.cos(sth)
FINAL = wrap(sth + math.radians(float(sys.argv[3]))) if len(sys.argv) > 3 else None
print(f"goal: {FWD} m forward, {LEFT} m left -> RViz x {GX:.3f} y {GY:.3f}", flush=True)
n.report("start")
try:
    x, y, th = n.pose()
    if math.hypot(x-GX, y-GY) > 0.02:
        bearing = math.atan2(GY-y, GX-x)
        reverse = abs(wrap(bearing - th)) > math.radians(150)
        heading = wrap(bearing + math.pi) if reverse else bearing
        n.turn_to(heading)
        t1 = time.time()
        while time.time()-t1 < 60:
            x, y, th = n.pose(); d = math.hypot(x-GX, y-GY)
            ahead = (GX-x)*math.cos(th) + (GY-y)*math.sin(th)   # distance to goal along the robot's axis
            if reverse: ahead = -ahead
            if ahead <= 0.005: break
            v = min(V, max(0.05, ahead))
            bearing = math.atan2(GY-y, GX-x); heading = wrap(bearing + math.pi) if reverse else bearing
            w = max(-0.3, min(0.3, 2.0*wrap(heading - th))) if d > 0.1 else 0.0
            n.send(-v if reverse else v, w); n.spin()
        n.stop(); n.report("arrived")
    if FINAL is not None: n.turn_to(FINAL)
finally:
    n.stop(1.0)
n.report("done")
x, y, th = n.pose(); c, s_ = math.cos(-sth), math.sin(-sth)
print(f"relative to start: {((x-sx)*c-(y-sy)*s_):.3f} m forward, {((x-sx)*s_+(y-sy)*c):.3f} m left, heading {math.degrees(wrap(th-sth)):.1f} deg")
rclpy.shutdown()
