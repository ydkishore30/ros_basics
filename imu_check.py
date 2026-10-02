import math, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import Log
rclpy.init(); n = Node('imu_check'); msgs = []; logs = []
n.create_subscription(Imu, '/imu', msgs.append, 50)
n.create_subscription(Log, '/rosout', lambda m: logs.append(m.msg) if 'MyHardware' in m.name or 'hardware' in m.name.lower() or 'Encoder' in m.msg else None, 50)
t0 = time.time()
while time.time()-t0 < 4: rclpy.spin_once(n, timeout_sec=0.05)
print(f"/imu: {len(msgs)} msgs in 4s = {len(msgs)/4:.1f} Hz")
if msgs:
    m = msgs[-1]; q = m.orientation
    roll = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))))
    yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    print(f"orientation: roll {roll:.1f}  pitch {pitch:.1f}  yaw {yaw:.1f} deg")
    print("angular_velocity z samples:", sorted(set(round(x.angular_velocity.z, 4) for x in msgs))[:8])
from collections import Counter
for k, v in Counter(logs).most_common(3): print(f"log x{v}: {k[:120]}")
rclpy.shutdown()
