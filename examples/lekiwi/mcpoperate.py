# !/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import (
    KeyboardTeleop,
    KeyboardTeleopConfig,
)
from lerobot.teleoperators.csvarm_leader import CsvArmLeader, CsvArmLeaderConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import log_rerun_data

from fastmcp import FastMCP

FPS = 30

# Create the robot and teleoperator configurations
robot_config = LeKiwiClientConfig(remote_ip="192.168.68.94", id="lekiwi")
teleop_arm_config = CsvArmLeaderConfig(port="arm.csv", id="csvarm_leader_arm")
keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

# Initialize the robot and teleoperator
robot = LeKiwiClient(robot_config)
leader_arm = CsvArmLeader(teleop_arm_config)
keyboard = KeyboardTeleop(keyboard_config)

# Connect to the robot and teleoperator
# To connect you already should have this script running on LeKiwi: `python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi`
robot.connect()
leader_arm.connect()
keyboard.connect()

leader_arm.grabber.robot = robot
leader_arm.grabber.teleop_arm = leader_arm

observation = robot.get_observation()
leader_arm.grabber.set_joint(observation["observation.state"][:6])

# Init rerun viewer
#init_rerun(session_name="lekiwi_teleop")

if not robot.is_connected or not leader_arm.is_connected:
    raise ValueError("Robot or teleop is not connected!")

lines = leader_arm.grabber.move_init()


mcp = FastMCP("MCP Server")


@mcp.tool
def get_robot_description() -> str:
    return leader_arm.grabber.robot_description


@mcp.tool
def move_forward():
    observation = robot.get_observation()
    arm_action = dict(list(observation.items())[:6])
    base_action = {
        "x.vel": 0.1,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }
    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
    _ = robot.send_action(action)
    return action


@mcp.tool
def move_backward():
    observation = robot.get_observation()
    arm_action = dict(list(observation.items())[:6])
    base_action = {
        "x.vel": -0.1,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }
    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
    _ = robot.send_action(action)
    return action


@mcp.tool
def move_right():
    observation = robot.get_observation()
    arm_action = dict(list(observation.items())[:6])
    base_action = {
        "x.vel": 0.0,
        "y.vel": -0.1,
        "theta.vel": 0.0,
    }
    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
    _ = robot.send_action(action)
    return action


@mcp.tool
def move_left():
    observation = robot.get_observation()
    arm_action = dict(list(observation.items())[:6])
    base_action = {
        "x.vel": 0.0,
        "y.vel": 0.1,
        "theta.vel": 0.0,
    }
    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
    _ = robot.send_action(action)
    return action


if __name__ == "__main__":
    mcp.run()
