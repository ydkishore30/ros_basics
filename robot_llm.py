#!/usr/bin/env python3
"""
Robot LLM — chat with an OpenAI model to run the REAL robot (Pi + Docker).

The model gets tools to check USB devices, launch/stop the robot bringup and
SLAM, save maps, drive the robot, check status and read logs. Everything runs
inside the ROS container via `docker exec`.

Setup (once):
    .venv/bin/pip install openai python-dotenv
    # ~/.env  must contain  OPENAI_API_KEY=sk-...

Usage:
    .venv/bin/python robot_llm.py                  # default model
    .venv/bin/python robot_llm.py --model gpt-4.1  # or set OPENAI_MODEL

Examples:
    > are the esp32 and lidar plugged in?
    > start the robot and slam
    > how is it doing?
    > move forward 0.5 m, then turn left 90 degrees
    > save the map as kitchen
    > stop everything
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

from dotenv import load_dotenv
from openai import OpenAI

# ── CONFIG ────────────────────────────────────────────────────────────────────
WORKSPACE = os.path.dirname(os.path.abspath(__file__))
CONTAINER = "ros_basic_container"
ROS_SETUP = "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash"
# Not under log/: colcon (running as root in the container) owns and wipes that folder
LOG_DIR = os.path.join(WORKSPACE, "robot_llm_logs")

# Fixed USB-port paths (same as robot.sh / my_robot_real.launch.py)
LIDAR_PORT = "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0"
ESP32_PORT = "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0"

# Launch commands, and the pattern used to find/stop each one in the container
LAUNCHES = {
    "robot": {
        "cmd": "ros2 launch my_robot_bringup my_robot_real.launch.py use_lidar:={use_lidar}",
        "pattern": "my_robot_real.launch.py",
        "ready": "Configured and activated diff_drive_controller",
    },
    "slam": {
        "cmd": "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False",
        "pattern": "online_async_launch.py",
        "ready": "Registering sensor",
    },
}

# Motion limits — diff_drive_controller stops the wheels if cmd_vel stops for
# 0.5s, so motion is streamed at CMD_VEL_RATE_HZ for the whole duration.
CMD_VEL_RATE_HZ = 20
MAX_LINEAR = 0.3     # m/s
MAX_ANGULAR = 1.0    # rad/s
MAX_DISTANCE = 2.0   # m per move command
MAX_ANGLE = 360.0    # deg per rotate command

# ros2 CLI sub-commands the model may run freely (read-only inspection).
ALLOWED_ROS2 = {
    "node": {"list", "info"},
    "topic": {"list", "info", "echo", "hz", "bw", "type", "find"},
    "service": {"list", "type", "find"},
    "param": {"list", "get", "describe"},
    "action": {"list", "info"},
    "interface": {"list", "show", "package", "packages"},
    "pkg": {"list", "prefix", "executables", "xml"},
    "lifecycle": {"list", "get", "nodes"},
    "control": {"list_controllers", "list_hardware_components", "list_hardware_interfaces"},
    "doctor": set(),
}

SYSTEM_PROMPT = f"""You are the operator assistant for a real differential-drive robot \
(Raspberry Pi + ESP32 motor/encoder board + YDLidar X2 + BNO IMU), running ROS 2 Jazzy \
inside the Docker container '{CONTAINER}'.

Rules:
- Before starting the robot, call check_devices. The ESP32 must be on USB port 1.3 and the \
lidar on port 1.2. If either is missing, tell the user to fix the cable — do not launch.
- SLAM needs the robot bringup running first (lidar scans + odometry). Start robot, then slam.
- Before stopping SLAM or everything, if a map was being built and the user has not saved \
it, remind them once and offer save_map. Only save if asked.
- A bare "stop" / "halt" / "brake" means stop_robot (stop driving), NOT stopping launches. \
Use stop_launch only when the user asks to stop the robot launch, SLAM, nodes or everything.
- Drive only with move / rotate / stop_robot. Keep answers short and practical.
- When something fails, read the relevant log with read_log and explain the cause.
"""

# ── HELPERS ───────────────────────────────────────────────────────────────────

def run(cmd: list[str], timeout: int = 15) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        out = (r.stdout + ("\n" + r.stderr if r.returncode else "")).strip()
        return out[-3000:] if out else "(no output)"
    except subprocess.TimeoutExpired:
        return f"(timed out after {timeout}s)"
    except Exception as e:
        return f"(error: {e})"


def container_running() -> bool:
    out = run(["docker", "inspect", "-f", "{{.State.Running}}", CONTAINER])
    return out.strip() == "true"


def ensure_container() -> str | None:
    """Start the container if needed. Returns an error string, or None if OK."""
    if container_running():
        return None
    run(["docker", "start", CONTAINER], timeout=60)
    return None if container_running() else f"Could not start container {CONTAINER}."


def docker_exec(args: list[str], timeout: int = 15) -> str:
    """Run a command in the container with ROS sourced.

    args are passed as positional parameters ("$@"), never interpolated into
    the shell string, so tool inputs cannot inject extra shell commands.
    """
    return run(["docker", "exec", CONTAINER, "bash", "-c", f'{ROS_SETUP} && exec "$@"', "ros"]
               + args, timeout=timeout)


def launch_running(name: str) -> bool:
    out = run(["docker", "exec", CONTAINER, "pgrep", "-f", LAUNCHES[name]["pattern"]])
    return any(line.strip().isdigit() for line in out.splitlines())


def log_path(name: str) -> str:
    return os.path.join(LOG_DIR, f"{name}.log")


def clean_log_lines(path: str) -> list[str]:
    """Log lines minus the harmless YDLidar 'Real points 271 > fixed points 270' spam."""
    try:
        with open(path, errors="replace") as f:
            return [l.rstrip() for l in f if "Real points" not in l]
    except FileNotFoundError:
        return []


DRIVE_SCRIPT = """
import sys, time, rclpy
from geometry_msgs.msg import Twist
lin, ang, dur, rate = map(float, sys.argv[1:5])
rclpy.init()
node = rclpy.create_node('robot_llm_drive')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 3.0:
    time.sleep(0.05)
if pub.get_subscription_count() == 0:
    print('No subscriber on /cmd_vel - is the robot launch running?'); sys.exit(1)
msg = Twist(); msg.linear.x = lin; msg.angular.z = ang
end = time.time() + dur
try:
    while time.time() < end:
        pub.publish(msg); time.sleep(1.0 / rate)
finally:
    for _ in range(5):
        pub.publish(Twist()); time.sleep(0.05)
print(f'Done: linear={lin} m/s angular={ang} rad/s for {dur:.2f}s')
"""


def drive(linear: float, angular: float, duration: float) -> str:
    if not launch_running("robot"):
        return "Robot launch is not running — start it first."
    return docker_exec(["python3", "-c", DRIVE_SCRIPT, str(linear), str(angular),
                        str(duration), str(CMD_VEL_RATE_HZ)], timeout=int(duration) + 20)


# ── TOOLS ─────────────────────────────────────────────────────────────────────

def check_devices() -> str:
    esp = os.path.exists(ESP32_PORT)
    lidar = os.path.exists(LIDAR_PORT)
    cp210x = sum("CP210" in l for l in run(["lsusb"]).splitlines())
    return (f"ESP32 (USB port 1.3): {'OK' if esp else 'MISSING'}\n"
            f"Lidar (USB port 1.2): {'OK' if lidar else 'MISSING'}\n"
            f"CP210x adapters seen by lsusb: {cp210x} (expected 2)")


def start_launch(name: str, use_lidar: bool = True, wait_s: int = 45) -> str:
    if name not in LAUNCHES:
        return f"Unknown launch '{name}'. Use 'robot' or 'slam'."
    if err := ensure_container():
        return err
    if launch_running(name):
        return f"{name} is already running."
    if name == "robot":
        if not os.path.exists(ESP32_PORT):
            return "ESP32 not found on USB port 1.3 — not launching. " + check_devices()
        if use_lidar and not os.path.exists(LIDAR_PORT):
            return "Lidar not found on USB port 1.2 — not launching. " + check_devices()
    if name == "slam" and not launch_running("robot"):
        return "Robot bringup is not running — SLAM needs it. Start 'robot' first."

    os.makedirs(LOG_DIR, exist_ok=True)
    cmd = LAUNCHES[name]["cmd"].format(use_lidar=str(use_lidar).lower())
    log = open(log_path(name), "w")
    # start_new_session: Ctrl+C in this chat must not kill the launch
    subprocess.Popen(["docker", "exec", CONTAINER, "bash", "-c", f"{ROS_SETUP} && cd /ros2_ws && {cmd}"],
                     stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                     start_new_session=True)

    ready = LAUNCHES[name]["ready"]
    deadline = time.time() + wait_s
    while time.time() < deadline:
        time.sleep(2)
        lines = clean_log_lines(log_path(name))
        text = "\n".join(lines)
        timeouts = text.count("Encoder read timed out")
        if timeouts >= 5:
            return (f"{name} started but the ESP32 is not answering ({timeouts} encoder "
                    "timeouts). Check the ESP32 power/cable. Last log lines:\n" + "\n".join(lines[-10:]))
        if "process has died" in text:
            return f"{name} launch had a process die:\n" + "\n".join(lines[-15:])
        if ready in text:
            return f"{name} is up. Log: {log_path(name)}"
    return (f"{name} did not report ready within {wait_s}s. Last log lines:\n"
            + "\n".join(clean_log_lines(log_path(name))[-15:]))


def stop_launch(name: str) -> str:
    targets = ["slam", "robot"] if name == "all" else [name]
    if any(t not in LAUNCHES for t in targets):
        return "Use 'robot', 'slam' or 'all'."
    msgs = []
    for t in targets:  # SLAM first, so it doesn't lose its inputs mid-shutdown
        if not launch_running(t):
            msgs.append(f"{t}: not running")
            continue
        run(["docker", "exec", CONTAINER, "pkill", "-INT", "-f", LAUNCHES[t]["pattern"]])
        for _ in range(20):
            time.sleep(0.5)
            if not launch_running(t):
                break
        msgs.append(f"{t}: {'stopped' if not launch_running(t) else 'still shutting down'}")
    return "\n".join(msgs)


def save_map(name: str = "my_map") -> str:
    if not launch_running("slam"):
        return "SLAM is not running — nothing to save."
    safe = "".join(c for c in name if c.isalnum() or c in "_-") or "my_map"
    out = docker_exec(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", f"/ros2_ws/{safe}"],
                      timeout=30)
    pgm = os.path.join(WORKSPACE, f"{safe}.pgm")
    if os.path.exists(pgm):
        return f"Saved {pgm} and {safe}.yaml"
    return "Map save may have failed:\n" + out[-800:]


def status() -> str:
    if not container_running():
        return f"Container {CONTAINER} is not running."
    lines = [f"robot launch: {'RUNNING' if launch_running('robot') else 'stopped'}",
             f"slam launch:  {'RUNNING' if launch_running('slam') else 'stopped'}"]
    if launch_running("robot"):
        topics = ["/scan", "/diff_drive_controller/odom", "/imu", "/odometry/filtered"]
        with ThreadPoolExecutor(len(topics)) as pool:
            outs = pool.map(lambda t: docker_exec(["timeout", "6", "ros2", "topic", "hz", t], timeout=15),
                            topics)
        for t, out in zip(topics, outs):
            rate = [l.split(":")[1].strip() for l in out.splitlines() if "average rate" in l]
            lines.append(f"{t}: {rate[-1] + ' Hz' if rate else 'NO DATA'}")
        timeouts = sum("Encoder read timed out" in l for l in clean_log_lines(log_path("robot")))
        if timeouts:
            lines.append(f"WARNING: {timeouts} encoder timeouts in robot log")
    return "\n".join(lines)


def read_log(name: str, lines: int = 40) -> str:
    if name not in LAUNCHES:
        return "Use 'robot' or 'slam'."
    content = clean_log_lines(log_path(name))
    if not content:
        return f"No log for {name} (only launches started from this script are logged)."
    return "\n".join(content[-min(lines, 150):])


def move(distance_m: float, speed: float = 0.15) -> str:
    distance_m = max(-MAX_DISTANCE, min(MAX_DISTANCE, distance_m))
    speed = max(0.05, min(MAX_LINEAR, abs(speed)))
    if distance_m == 0:
        return "Distance is 0 — nothing to do."
    lin = speed if distance_m > 0 else -speed
    return drive(lin, 0.0, abs(distance_m) / speed)


def rotate(direction: str = "left", angle_deg: float = 90.0, speed: float = 0.5) -> str:
    angle = max(0.0, min(MAX_ANGLE, abs(angle_deg)))
    speed = max(0.1, min(MAX_ANGULAR, abs(speed)))
    ang = speed if direction.lower().startswith("l") else -speed
    return drive(0.0, ang, (angle * 3.14159265 / 180.0) / speed)


def stop_robot() -> str:
    if not launch_running("robot"):
        return "Robot launch is not running."
    return drive(0.0, 0.0, 0.3)


def ros2_cmd(command: str) -> str:
    try:
        args = shlex.split(command)
    except ValueError as e:
        return f"(could not parse command: {e})"
    if args and args[0] == "ros2":
        args = args[1:]
    if not args or args[0] not in ALLOWED_ROS2:
        return f"Not allowed. Allowed: {', '.join(sorted(ALLOWED_ROS2))}"
    subs = ALLOWED_ROS2[args[0]]
    if subs and (len(args) < 2 or args[1] not in subs):
        return f"Allowed 'ros2 {args[0]}' sub-commands: {', '.join(sorted(subs))}"
    # echo / hz / bw never exit on their own
    if args[:2] in (["topic", "echo"],) and "--once" not in args:
        args.append("--once")
    prefix = ["timeout", "8"] if args[:2] in (["topic", "hz"], ["topic", "bw"], ["topic", "echo"]) else []
    return docker_exec(prefix + ["ros2"] + args, timeout=20)


TOOLS = {
    "check_devices": (check_devices, "Check the ESP32 (USB port 1.3) and lidar (USB port 1.2) are connected.", {}),
    "start_launch": (start_launch, "Start the 'robot' bringup or 'slam' and wait until ready.", {
        "name": {"type": "string", "enum": ["robot", "slam"]},
        "use_lidar": {"type": "boolean", "description": "robot only; default true"},
    }),
    "stop_launch": (stop_launch, "Cleanly stop (Ctrl+C) the 'robot' launch, 'slam', or 'all'.", {
        "name": {"type": "string", "enum": ["robot", "slam", "all"]},
    }),
    "save_map": (save_map, "Save the current SLAM map to ~/ros_basics/<name>.pgm/.yaml.", {
        "name": {"type": "string", "description": "file name without extension"},
    }),
    "status": (status, "Which launches are running, sensor/odometry rates, encoder errors.", {}),
    "read_log": (read_log, "Tail the log of a launch started from this script (lidar spam filtered).", {
        "name": {"type": "string", "enum": ["robot", "slam"]},
        "lines": {"type": "integer"},
    }),
    "move": (move, f"Drive straight. Negative distance = backwards. Max {MAX_DISTANCE} m, {MAX_LINEAR} m/s.", {
        "distance_m": {"type": "number"},
        "speed": {"type": "number", "description": "m/s, default 0.15"},
    }),
    "rotate": (rotate, "Rotate in place.", {
        "direction": {"type": "string", "enum": ["left", "right"]},
        "angle_deg": {"type": "number"},
        "speed": {"type": "number", "description": "rad/s, default 0.5"},
    }),
    "stop_robot": (stop_robot, "Stop driving immediately (zero velocity). Does not stop launches.", {}),
    "ros2_cmd": (ros2_cmd, "Read-only ros2 CLI inspection, e.g. 'node list', 'topic echo /imu', "
                           "'control list_controllers'. No pub/call/set/run/launch.", {
        "command": {"type": "string"},
    }),
}

REQUIRED = {"start_launch": ["name"], "stop_launch": ["name"], "read_log": ["name"],
            "move": ["distance_m"], "rotate": ["direction"], "ros2_cmd": ["command"]}

TOOL_SPECS = [
    {"type": "function", "function": {
        "name": name, "description": desc,
        "parameters": {"type": "object", "properties": props, "required": REQUIRED.get(name, [])},
    }}
    for name, (_, desc, props) in TOOLS.items()
]


def call_tool(name: str, raw_args: str) -> str:
    if name not in TOOLS:
        return f"Unknown tool {name}"
    try:
        args = json.loads(raw_args or "{}")
        return str(TOOLS[name][0](**args))
    except Exception as e:
        return f"Tool error: {e}"


# ── CHAT LOOP ─────────────────────────────────────────────────────────────────

def chat_turn(client: OpenAI, model: str, messages: list) -> None:
    while True:
        resp = client.chat.completions.create(model=model, messages=messages, tools=TOOL_SPECS)
        msg = resp.choices[0].message
        messages.append(msg.model_dump(exclude_none=True))
        if not msg.tool_calls:
            print(f"\nrobot> {msg.content}\n")
            return
        for tc in msg.tool_calls:
            print(f"  [{tc.function.name} {tc.function.arguments}]")
            result = call_tool(tc.function.name, tc.function.arguments)
            messages.append({"role": "tool", "tool_call_id": tc.id, "content": result})


def drop_turn(messages: list) -> None:
    """Remove an unfinished turn so the history stays valid for the API."""
    while len(messages) > 1 and messages[-1].get("role") != "user":
        messages.pop()
    if len(messages) > 1:
        messages.pop()


def main():
    load_dotenv(os.path.expanduser("~/.env"))
    load_dotenv(os.path.join(WORKSPACE, ".env"))

    parser = argparse.ArgumentParser(description="Chat with an LLM to run the real robot.")
    parser.add_argument("--model", default=os.getenv("OPENAI_MODEL", "gpt-4.1-mini"))
    args = parser.parse_args()

    if not os.getenv("OPENAI_API_KEY"):
        sys.exit("OPENAI_API_KEY not found in ~/.env or the environment.")
    client = OpenAI()
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    print(f"Robot LLM ({args.model}). Type 'quit' to exit. Ctrl+C while driving stops the robot.\n")
    while True:
        try:
            user = input("you> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not user:
            continue
        if user.lower() in ("quit", "exit", "q"):
            break
        messages.append({"role": "user", "content": user})
        try:
            chat_turn(client, args.model, messages)
        except KeyboardInterrupt:
            print("\n  [interrupted — stopping the robot]")
            print("  " + stop_robot())
            drop_turn(messages)
        except Exception as e:
            print(f"\n  [error: {e}]\n")
            drop_turn(messages)

    running = [n for n in LAUNCHES if container_running() and launch_running(n)]
    if running:
        ans = input(f"{', '.join(running)} still running. Stop them? [y/N] ").strip().lower()
        if ans == "y":
            print(stop_launch("all"))


if __name__ == "__main__":
    main()
