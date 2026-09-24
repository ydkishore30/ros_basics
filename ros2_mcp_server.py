#!/usr/bin/env python3
"""
ROS2 MCP Server — exposes robot control tools over the Model Context Protocol.

Any MCP client (ros2_agent.py + Ollama, Claude Code, etc.) can connect to it.

Usage:
    # stdio (default) — the client spawns this process itself
    python3 ros2_mcp_server.py

    # HTTP — lets clients on other machines (e.g. your laptop) connect
    python3 ros2_mcp_server.py --transport http --host 0.0.0.0 --port 8765
    # endpoint: http://<pi-ip>:8765/mcp
"""

import argparse
import math
import os
import shlex
import socket
import subprocess
import time

from mcp.server.fastmcp import FastMCP
from mcp.server.transport_security import TransportSecuritySettings

# ── CONFIG ────────────────────────────────────────────────────────────────────
WORKSPACE = os.path.dirname(os.path.abspath(__file__))
ROS_SETUP = (
    "source /opt/ros/jazzy/setup.bash && "
    "{ source /ros2_ws/install/setup.bash 2>/dev/null || true; }"
)

INSTRUCTIONS = """Controls a differential-drive ROS2 robot (Gazebo sim or real Pi hardware)
with Nav2 navigation and AprilTag docking.
- Robot spawns at map position (0, 0) facing +X.
- Dock station (home_dock) is at map position (2.5, 0.0)."""

mcp = FastMCP("ros2", instructions=INSTRUCTIONS)


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


def docker_exec(container: str, args: list[str], timeout: int = 10) -> str:
    """Run a command inside the container with ROS sourced.

    args are passed as positional parameters ("$@"), never interpolated into
    the shell string, so tool inputs cannot inject extra shell commands.
    """
    script = f'{ROS_SETUP} && exec "$@"'
    try:
        result = subprocess.run(
            ["docker", "exec", container, "bash", "-c", script, "ros"] + args,
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


NO_CONTAINER = "No running ROS container found. Start the simulation/robot first."


# ── TOOLS ─────────────────────────────────────────────────────────────────────

@mcp.tool()
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

    for _ in range(30):
        time.sleep(2)
        c = get_container()
        if c:
            return f"Simulation started in container {c}. Nav2 may take ~20s more to fully activate."
    return "Simulation launch initiated but container not detected yet — it may still be loading."


@mcp.tool()
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


@mcp.tool()
def ros2_cmd(command: str, timeout: int = 15) -> str:
    """Run a ros2 CLI command inside the container and return stdout.
    Do NOT include 'ros2' prefix — e.g. 'topic list', 'node list', 'topic echo /scan --once'."""
    container = get_container()
    if not container:
        return NO_CONTAINER
    try:
        args = shlex.split(command)
    except ValueError as e:
        return f"(could not parse command: {e})"
    if args and args[0] == "ros2":
        args = args[1:]
    return docker_exec(container, ["ros2"] + args, timeout=min(timeout, 60))


@mcp.tool()
def navigate(x: float, y: float, yaw: float = 0.0) -> str:
    """Send a Nav2 NavigateToPose goal to drive the robot to (x, y) with optional yaw (radians)."""
    container = get_container()
    if not container:
        return NO_CONTAINER
    qz = round(math.sin(yaw / 2), 4)
    qw = round(math.cos(yaw / 2), 4)
    goal = (
        f"{{pose: {{header: {{frame_id: 'map'}}, "
        f"pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}}}"
    )
    return docker_exec(
        container,
        ["ros2", "action", "send_goal", "/navigate_to_pose",
         "nav2_msgs/action/NavigateToPose", goal],
        timeout=60,
    )


@mcp.tool()
def dock() -> str:
    """Trigger autonomous docking to the home_dock AprilTag station."""
    container = get_container()
    if not container:
        return NO_CONTAINER
    return docker_exec(container, ["ros2", "run", "my_robot_navigation", "dock_robot"], timeout=120)


@mcp.tool()
def robot_pose() -> str:
    """Get the current robot position from /amcl_pose."""
    container = get_container()
    if not container:
        return NO_CONTAINER
    raw = docker_exec(container, ["ros2", "topic", "echo", "/amcl_pose", "--once"], timeout=15)
    pose_lines = [l for l in raw.splitlines() if "x:" in l or "y:" in l or "z:" in l]
    return "\n".join(pose_lines[:6]) if pose_lines else raw


@mcp.tool()
def node_list() -> str:
    """List all active ROS2 nodes."""
    container = get_container()
    if not container:
        return NO_CONTAINER
    result = docker_exec(container, ["ros2", "node", "list"], timeout=15)
    if result in ("(no output)", ""):
        return "No ROS2 nodes found yet — container may still be starting up. Try again in a few seconds."
    return result


@mcp.tool()
def topic_list() -> str:
    """List all active ROS2 topics."""
    container = get_container()
    if not container:
        return NO_CONTAINER
    return docker_exec(container, ["ros2", "topic", "list"], timeout=15)


# ── ENTRY POINT ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ROS2 MCP server")
    parser.add_argument("--transport", choices=["stdio", "http"], default="stdio")
    parser.add_argument("--host", default="127.0.0.1",
                        help="HTTP bind address (use 0.0.0.0 to allow LAN clients)")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    if args.transport == "http":
        mcp.settings.host = args.host
        mcp.settings.port = args.port
        # Accept Host headers naming this machine (LAN IPs, hostname) while
        # keeping DNS-rebinding protection against arbitrary hostnames.
        ips = subprocess.run(["hostname", "-I"], capture_output=True, text=True).stdout.split()
        names = ["localhost", "127.0.0.1", socket.gethostname(), f"{socket.gethostname()}.local", *ips]
        mcp.settings.transport_security = TransportSecuritySettings(
            enable_dns_rebinding_protection=True,
            allowed_hosts=[f"{n}:{args.port}" for n in names] + [f"[{ip}]:{args.port}" for ip in ips if ":" in ip],
        )
        mcp.run(transport="streamable-http")
    else:
        mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
