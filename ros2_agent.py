#!/usr/bin/env python3
"""
ROS2 AI Agent — Natural language control of the robot via LangGraph + Ollama.

Usage:
    python3 ros2_agent.py [--model llama3.2:3b]

Examples:
    > start the simulation with nav2 and docking
    > navigate to position 3, 2
    > dock the robot
    > what nodes are running?
    > stop sim
"""

import argparse
import math
import subprocess
import sys
import time
import warnings

import requests
from langchain_core.messages import HumanMessage, ToolMessage
from langchain_core.tools import tool
from langchain_ollama import ChatOllama
from langgraph.prebuilt import create_react_agent

warnings.filterwarnings("ignore", category=DeprecationWarning)

# ── CONFIG ────────────────────────────────────────────────────────────────────
import os
WORKSPACE = os.path.dirname(os.path.abspath(__file__))

SYSTEM_PROMPT = """You are a ROS2 robot assistant that controls a differential-drive robot \
running in Gazebo simulation with Nav2 navigation and AprilTag docking.

Robot facts:
- Spawns at map position (0, 0) facing +X direction
- Dock station (home_dock) is at map position (2.5, 0.0)
- AprilTag marker faces the robot at that position

IMPORTANT intent mapping — always call sim_stop when the user says any of:
"stop", "stop sim", "stop all sim", "stop simulation", "shut down", "bring down",
"bring down sim", "kill", "kill sim", "kill all", "kill all nodes", "kill nodes",
"kill everything", "exit sim", "close", "terminate", "bring the launch file down",
"stop the launch", "down all", "stop all", "halt", "sim_stop".
Do NOT call node_list for "kill all nodes" — that means stop everything, use sim_stop.

Always call sim_start when the user says any of:
"start", "launch", "bring up", "run", "open", "spin up".

When asked to do something, pick the right tool and call it.
Report what happened after each tool call.
If a command fails, explain why and suggest a fix."""


# ── HELPERS ───────────────────────────────────────────────────────────────────

def get_container() -> str | None:
    result = subprocess.run(
        ["docker", "ps", "--format", "{{.Names}}"],
        capture_output=True, text=True,
    )
    for name in result.stdout.splitlines():
        if "ros" in name:
            return name
    return None


def docker_exec(container: str, cmd: str, timeout: int = 10) -> str:
    full = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source /ros2_ws/install/setup.bash 2>/dev/null || true && "
        f"{cmd}"
    )
    try:
        result = subprocess.run(
            ["docker", "exec", container, "bash", "-c", full],
            capture_output=True, text=True, timeout=timeout,
        )
        if result.returncode == 0:
            out = result.stdout.strip()
        else:
            out = (result.stderr or result.stdout).strip()
        return out[:2000] if out else "(no output)"
    except subprocess.TimeoutExpired:
        return f"(timed out after {timeout}s)"
    except Exception as e:
        return f"(error: {e})"


# ── TOOLS ─────────────────────────────────────────────────────────────────────

@tool
def sim_start(use_nav2: bool = True, use_slam: bool = False, use_dock: bool = True) -> str:
    """Start, launch, bring up, or spin up the Gazebo simulation with Nav2 and docking.
    Use this when the user says: start, launch, bring up, open, run, spin up the sim.
    Pass use_slam=True to run SLAM instead of a pre-built map."""
    container = get_container()
    if container:
        return f"Simulation already running in container: {container}"

    args = (
        f"use_nav2:={'true' if use_nav2 else 'false'} "
        f"use_slam:={'true' if use_slam else 'false'} "
        f"use_dock:={'true' if use_dock else 'false'}"
    )
    cmd = (
        f"docker compose --profile pc run --rm "
        f"-e DISPLAY=${{DISPLAY:-:1}} ros-pc bash -c \""
        f"source /opt/ros/jazzy/setup.bash && "
        f"source /ros2_ws/install/setup.bash && "
        f"ros2 launch my_robot_bringup my_robot_gazebo.launch.py {args}\""
    )
    subprocess.Popen(cmd, shell=True, cwd=WORKSPACE,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("  [agent] Waiting for simulation to start", end="", flush=True)
    for _ in range(30):
        time.sleep(2)
        print(".", end="", flush=True)
        c = get_container()
        if c:
            print()
            return f"Simulation started in container {c}. Nav2 may take ~20s more to fully activate."
    print()
    return "Simulation launch initiated but container not detected yet — it may still be loading."


@tool
def sim_stop() -> str:
    """Stop, shut down, bring down, kill, or halt the simulation and all containers.
    Use this whenever the user says: stop, stop sim, stop all sim, stop simulation,
    shut down, bring down, bring down sim, kill, kill sim, kill all, kill all nodes,
    kill nodes, kill everything, close, terminate, stop all, down all, stop the launch,
    bring the launch file down, exit simulation, or similar."""
    ps = subprocess.run(
        ["docker", "ps", "--format", "{{.Names}}"],
        capture_output=True, text=True,
    )
    containers = [c for c in ps.stdout.splitlines() if "ros" in c]
    if not containers:
        return "No running ROS containers found."
    subprocess.run(["docker", "stop"] + containers, capture_output=True)
    return f"Stopped: {', '.join(containers)}"


@tool
def ros2_cmd(command: str, timeout: int = 5) -> str:
    """Run a ros2 CLI command inside the container and return stdout.
    Do NOT include 'ros2' prefix — e.g. 'topic list', 'node list', 'topic echo /scan --once'."""
    container = get_container()
    if not container:
        return "No running simulation container found. Start the simulation first."
    return docker_exec(container, f"ros2 {command}", timeout=timeout)


@tool
def navigate(x: float, y: float, yaw: float = 0.0) -> str:
    """Send a Nav2 NavigateToPose goal to drive the robot to (x, y) with optional yaw (radians)."""
    container = get_container()
    if not container:
        return "No running simulation container found. Start the simulation first."
    qz = round(math.sin(yaw / 2), 4)
    qw = round(math.cos(yaw / 2), 4)
    goal = (
        f"{{pose: {{header: {{frame_id: 'map'}}, "
        f"pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}}}"
    )
    cmd = f"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{goal}\""
    return docker_exec(container, cmd, timeout=60)


@tool
def dock() -> str:
    """Trigger autonomous docking to the home_dock AprilTag station."""
    container = get_container()
    if not container:
        return "No running simulation container found. Start the simulation first."
    return docker_exec(container, "ros2 run my_robot_navigation dock_robot", timeout=120)


@tool
def robot_pose() -> str:
    """Get the current robot position from /amcl_pose."""
    container = get_container()
    if not container:
        return "No running simulation container found."
    raw = docker_exec(container, "ros2 topic echo /amcl_pose --once", timeout=6)
    lines = raw.splitlines()
    pose_lines = [l for l in lines if "x:" in l or "y:" in l or "z:" in l]
    return "\n".join(pose_lines[:6]) if pose_lines else raw


@tool
def node_list() -> str:
    """List all active ROS2 nodes."""
    container = get_container()
    if not container:
        return "No running simulation container found."
    result = docker_exec(container, "ros2 node list", timeout=5)
    if result in ("(no output)", ""):
        return "No ROS2 nodes found yet — container may still be starting up. Try again in a few seconds."
    return result


@tool
def topic_list() -> str:
    """List all active ROS2 topics."""
    container = get_container()
    if not container:
        return "No running simulation container found."
    return docker_exec(container, "ros2 topic list", timeout=5)


TOOLS = [sim_start, sim_stop, ros2_cmd, navigate, dock, robot_pose, node_list, topic_list]


# ── LANGGRAPH AGENT ───────────────────────────────────────────────────────────

def run_agent(model: str):
    print(f"\n ROS2 Agent  (model: {model}, powered by LangGraph)")
    print(" Type your command in plain English. 'quit' to exit.\n")

    llm = ChatOllama(model=model, base_url="http://localhost:11434")
    agent = create_react_agent(llm, TOOLS, prompt=SYSTEM_PROMPT)

    while True:
        try:
            user_input = input("You: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye.")
            break

        if not user_input:
            continue
        if user_input.lower() in ("quit", "exit", "q"):
            break

        try:
            result = agent.invoke({"messages": [HumanMessage(content=user_input)]})
            # Print tool calls made during this turn
            for msg in result["messages"]:
                if isinstance(msg, ToolMessage):
                    print(f"  [tool:{msg.name}] {str(msg.content)[:300]}")
            # Last message is the final AI response
            final = result["messages"][-1]
            print(f"\nAgent: {final.content.strip()}\n")
        except Exception as e:
            print(f"\n[error] {e}\n")


# ── ENTRY POINT ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ROS2 AI Agent via LangGraph + Ollama")
    parser.add_argument("--model", default="llama3.2:3b",
                        help="Ollama model to use (default: llama3.2:3b)")
    args = parser.parse_args()

    try:
        r = requests.get("http://localhost:11434/api/tags", timeout=3)
        models = [m["name"] for m in r.json().get("models", [])]
        if args.model not in models:
            print(f"[warn] Model '{args.model}' not found. Available: {', '.join(models)}")
            if models:
                args.model = models[0]
                print(f"[info] Using '{args.model}' instead.\n")
    except Exception:
        print("[error] Ollama not reachable at localhost:11434")
        sys.exit(1)

    run_agent(args.model)


if __name__ == "__main__":
    main()
