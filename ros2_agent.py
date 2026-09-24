#!/usr/bin/env python3
"""
ROS2 AI Agent — Natural language control of the robot via LangGraph + Ollama,
with robot tools served by the ROS2 MCP server (ros2_mcp_server.py).

Usage:
    # Ollama on your laptop, MCP server spawned locally over stdio
    python3 ros2_agent.py --ollama-url http://<laptop-ip>:11434

    # Connect to an MCP server already running over HTTP (e.g. on the Pi)
    python3 ros2_agent.py --mcp-url http://<pi-ip>:8765/mcp

    OLLAMA_HOST / ROS2_MCP_URL env vars work in place of the flags.

Examples:
    > start the simulation with nav2 and docking
    > navigate to position 3, 2
    > dock the robot
    > what nodes are running?
    > stop sim
"""

import argparse
import asyncio
import os
import sys
import warnings

import requests
from langchain_core.messages import HumanMessage, ToolMessage
from langchain_mcp_adapters.client import MultiServerMCPClient
from langchain_ollama import ChatOllama
from langgraph.prebuilt import create_react_agent

warnings.filterwarnings("ignore", category=DeprecationWarning)

# ── CONFIG ────────────────────────────────────────────────────────────────────
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


# ── LANGGRAPH AGENT ───────────────────────────────────────────────────────────

def mcp_connection(mcp_url: str | None) -> dict:
    if mcp_url:
        return {"transport": "streamable_http", "url": mcp_url}
    return {
        "transport": "stdio",
        "command": sys.executable,
        "args": [os.path.join(WORKSPACE, "ros2_mcp_server.py")],
    }


async def run_agent(model: str, ollama_url: str, mcp_url: str | None):
    client = MultiServerMCPClient({"ros2": mcp_connection(mcp_url)})
    tools = await client.get_tools()

    print(f"\n ROS2 Agent  (model: {model} @ {ollama_url}, powered by LangGraph)")
    print(f" MCP server: {mcp_url or 'ros2_mcp_server.py (stdio)'}")
    print(f" Tools: {', '.join(t.name for t in tools)}")
    print(" Type your command in plain English. 'quit' to exit.\n")

    llm = ChatOllama(model=model, base_url=ollama_url)
    agent = create_react_agent(llm, tools, prompt=SYSTEM_PROMPT)

    while True:
        try:
            user_input = (await asyncio.to_thread(input, "You: ")).strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye.")
            break

        if not user_input:
            continue
        if user_input.lower() in ("quit", "exit", "q"):
            break

        try:
            result = await agent.ainvoke({"messages": [HumanMessage(content=user_input)]})
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
    parser = argparse.ArgumentParser(description="ROS2 AI Agent via LangGraph + Ollama + MCP")
    parser.add_argument("--model", default="llama3.2:3b",
                        help="Ollama model to use (default: llama3.2:3b)")
    parser.add_argument("--ollama-url", default=os.environ.get("OLLAMA_HOST", "http://localhost:11434"),
                        help="Ollama base URL, e.g. http://192.168.29.22:11434 (env: OLLAMA_HOST)")
    parser.add_argument("--mcp-url", default=os.environ.get("ROS2_MCP_URL"),
                        help="ROS2 MCP server HTTP URL; omit to spawn it over stdio (env: ROS2_MCP_URL)")
    args = parser.parse_args()

    ollama_url = args.ollama_url.rstrip("/")
    if not ollama_url.startswith("http"):
        ollama_url = f"http://{ollama_url}"

    try:
        r = requests.get(f"{ollama_url}/api/tags", timeout=3)
        models = [m["name"] for m in r.json().get("models", [])]
        if args.model not in models:
            print(f"[warn] Model '{args.model}' not found. Available: {', '.join(models)}")
            if models:
                args.model = models[0]
                print(f"[info] Using '{args.model}' instead.\n")
    except Exception:
        print(f"[error] Ollama not reachable at {ollama_url}")
        sys.exit(1)

    asyncio.run(run_agent(args.model, ollama_url, args.mcp_url))


if __name__ == "__main__":
    main()
