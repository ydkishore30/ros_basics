#!/usr/bin/env python3
"""Browser joystick for the robot: open http://<pi-ip>:8080 and drag the stick.

Publishes geometry_msgs/Twist on /cmd_vel. If the browser stops sending
(tab closed, wifi drop) the robot is commanded to zero after 0.4 s.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist

PORT = 8080
MAX_LIN = 0.20   # m/s
MAX_ANG = 0.35   # rad/s - at 0.8 the wheel odometry under-counts turns by ~8% and smears the SLAM map
STALE = 0.4      # s without a browser message -> stop

state = {'v': 0.0, 'w': 0.0, 't': 0.0}

PAGE = b"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Robot Joystick</title><style>
body{margin:0;font-family:sans-serif;background:#16181d;color:#e8e8e8;text-align:center;touch-action:none}
h3{margin:14px 0 4px}#pad{width:300px;height:300px;border-radius:50%;background:#262a33;border:2px solid #4a5060;
margin:14px auto;position:relative;touch-action:none}
#knob{width:90px;height:90px;border-radius:50%;background:#3d8bfd;position:absolute;left:105px;top:105px;pointer-events:none}
#stop{font-size:22px;padding:14px 60px;border:0;border-radius:10px;background:#d9363e;color:#fff}
#out{font-family:monospace;margin:10px}small{color:#9aa0ad}
</style></head><body><h3>Robot Joystick</h3>
<small>drag: up/down = forward/back, left/right = turn. Release to stop. Keys: W A S D / arrows, space = stop</small>
<div id=pad><div id=knob></div></div><div id=out>v 0.00 m/s &nbsp; w 0.00 rad/s</div>
<button id=stop>STOP</button> <div id=err style="color:#ff8080;margin:8px"></div>
<script>
const pad=document.getElementById('pad'),knob=document.getElementById('knob'),out=document.getElementById('out'),err=document.getElementById('err');
const R=105;let x=0,y=0,held=false,keys={};
function setk(){knob.style.left=(105+x*R)+'px';knob.style.top=(105-y*R)+'px'}
function mv(e){const b=pad.getBoundingClientRect();let dx=(e.clientX-b.left-150)/R,dy=-(e.clientY-b.top-150)/R;
 const m=Math.hypot(dx,dy);if(m>1){dx/=m;dy/=m}x=dx;y=dy;setk()}
function rel(){held=false;x=0;y=0;setk();send()}
pad.onpointerdown=e=>{held=true;pad.setPointerCapture(e.pointerId);mv(e)};
pad.onpointermove=e=>{if(held)mv(e)};pad.onpointerup=rel;pad.onpointercancel=rel;
document.getElementById('stop').onclick=()=>{keys={};rel()};
const K={ArrowUp:[0,1],w:[0,1],ArrowDown:[0,-1],s:[0,-1],ArrowLeft:[-1,0],a:[-1,0],ArrowRight:[1,0],d:[1,0]};
function kb(){let kx=0,ky=0;for(const k in keys){kx+=K[k][0];ky+=K[k][1]}x=kx*0.6;y=ky*0.6;setk()}
onkeydown=e=>{if(e.key==' '){keys={};rel();return}if(K[e.key]){keys[e.key]=1;kb();e.preventDefault()}};
onkeyup=e=>{if(K[e.key]){delete keys[e.key];kb();if(!Object.keys(keys).length)send()}};
onblur=()=>{keys={};rel()};
function send(){fetch('/cmd',{method:'POST',body:JSON.stringify({x:x,y:y})}).then(r=>r.json()).then(j=>{
 out.innerHTML='v '+j.v.toFixed(2)+' m/s &nbsp; w '+j.w.toFixed(2)+' rad/s';err.textContent=''}).catch(()=>{err.textContent='no connection to robot'})}
setInterval(()=>{if(held||Object.keys(keys).length)send()},100);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        self.wfile.write(PAGE)

    def do_POST(self):
        try:
            d = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
            x = max(-1.0, min(1.0, float(d['x'])))
            y = max(-1.0, min(1.0, float(d['y'])))
        except Exception:
            x = y = 0.0
        # stick right (x>0) = turn right = negative angular z
        state.update(v=y * MAX_LIN, w=-x * MAX_ANG, t=time.monotonic())
        body = json.dumps({'v': state['v'], 'w': state['w']}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(body)


def main():
    rclpy.init()
    node = rclpy.create_node('web_joystick')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    zeros_left = [0]

    def tick():
        msg = Twist()
        if time.monotonic() - state['t'] < STALE:
            msg.linear.x, msg.angular.z = state['v'], state['w']
            zeros_left[0] = 20
            pub.publish(msg)
        elif zeros_left[0] > 0:   # send zeros for 1 s, then stay quiet
            zeros_left[0] -= 1
            pub.publish(msg)

    node.create_timer(0.05, tick)
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f'web joystick on port {PORT}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())
        srv.shutdown()


if __name__ == '__main__':
    main()
