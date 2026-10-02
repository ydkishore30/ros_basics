import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log
rclpy.init(); n = Node('gap_probe'); st=[]; pos=[]; logs=[]
n.create_subscription(JointState,'/joint_states',lambda m:(st.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9),pos.append(tuple(m.position))),100)
n.create_subscription(Log,'/rosout',lambda m:logs.append(f"[{m.name}] {m.msg}"),100)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.05)
g=[(b-a)*1000 for a,b in zip(st,st[1:])]
print(f"msgs {len(st)} in 5s; unique stamps {len(set(st))}; unique positions {len(set(pos))}")
if g:
    import statistics as s
    print(f"gap ms: min {min(g):.1f} median {s.median(g):.1f} max {max(g):.1f}; gaps>50ms: {sum(x>50 for x in g)}; gaps<5ms: {sum(x<5 for x in g)}")
from collections import Counter
for k,v in Counter(logs).most_common(6): print(v,"x",k[:150])
