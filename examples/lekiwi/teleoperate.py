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
#from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.csvfile_leader import CsvFileLeader, CsvFileLeaderConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import log_rerun_data

FPS = 30

# Create the robot and teleoperator configurations
robot_config = LeKiwiClientConfig(remote_ip="192.168.68.94", id="lekiwi")
teleop_arm_config = CsvFileLeaderConfig(port="arm.csv", id="csvfile_leader_arm")
#keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

# Initialize the robot and teleoperator
robot = LeKiwiClient(robot_config)
leader_arm = CsvFileLeader(teleop_arm_config)
#keyboard = KeyboardTeleop(keyboard_config)

# Connect to the robot and teleoperator
# To connect you already should have this script running on LeKiwi: `python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi`
robot.connect()
leader_arm.connect()
#keyboard.connect()

leader_arm.grabber.robot = robot
leader_arm.grabber.teleop_arm = leader_arm

observation = robot.get_observation()
leader_arm.grabber.set_joint(observation["observation.state"][:6])

# Init rerun viewer
#init_rerun(session_name="lekiwi_teleop")

if not robot.is_connected or not leader_arm.is_connected:
    raise ValueError("Robot or teleop is not connected!")

print("Starting teleop loop...")
while True:
    t0 = time.perf_counter()

    # Get robot observation
    observation = robot.get_observation()

    # Get teleop action
    # Arm
    arm_action = leader_arm.get_action()
    arm_action = {f"arm_{k}": v for k, v in arm_action.items()}
    # Keyboard
    #keyboard_keys = keyboard.get_action()
    base_action = robot._from_twist_to_base_action()
    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

    # Send action to robot
    _ = robot.send_action(action)

    # Visualize
    #log_rerun_data(observation=observation, action=action)

    busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
