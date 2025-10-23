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
import cv2
import numpy as np
from ikmove import Ikmove

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30

# Recorded physical arm starting posture (lowered / tucked above base)
# [0.0, -91.6, 92.8, 59.0, 0.0, 0.0] in LeRobot degrees
START_DEG = [0.0, -91.6, 92.8, 59.0, 0.0, 0.0]

# Create the robot and teleoperator configurations
robot_config = LeKiwiClientConfig(remote_ip="10.0.0.71", id="lekiwi")

# Initialize the robot and teleoperator
robot = LeKiwiClient(robot_config)

# Connect to the robot and teleoperator
# To connect you already should have this script running on LeKiwi: `python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi`
robot.connect()

if not robot.is_connected:
    raise ValueError("Robot or teleop is not connected!")

# Read initial state and smoothly lower/transition the arm to standard rest pose
observation = robot.get_observation()
initial_arm_joints = [float(observation["observation.state"][i]) for i in range(6)]

print(f"Transitioning arm from current pose {np.round(initial_arm_joints, 1).tolist()} to starting pose {START_DEG}...")
ik = Ikmove()
# Generate smooth cubic spline over 2.0 seconds (60 frames at 30 fps)
traj_steps = max(int(np.max(np.abs(np.array(initial_arm_joints) - np.array(START_DEG)))), 60)
traj = ik.joint_trajectory([initial_arm_joints, START_DEG], [traj_steps])

joint_names = [
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
]

base_zero = {
    "x.vel": 0.0,
    "y.vel": 0.0,
    "theta.vel": 0.0,
}

for q in traj["joint_positions"]:
    t0 = time.perf_counter()
    arm_action = {name: float(val) for name, val in zip(joint_names, q)}
    robot.send_action({**arm_action, **base_zero})
    precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

print("Arm reached position. Disabling arm torque...")
time.sleep(0.2)
robot.disable_arm_torque()
print("Arm torque disabled. Starting teleop loop...")

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")
keyboard = KeyboardTeleop(keyboard_config)
keyboard.connect()

print("Teleoperation active! You can drive via:")
print("  1. ROS 2 topic /cmd_vel (e.g. from Nav2, joy, or teleop_twist_keyboard)")
print("  2. Direct keyboard in this window: W (forward), S (back), A (left), D (right), Z/X (turn), R/F (speed)")

while True:
    t0 = time.perf_counter()

    # Get robot observation and display camera feeds (front and wrist)
    observation = robot.get_observation()
    displayed_any = False
    for cam_key in ["front", "wrist"]:
        if cam_key in observation and observation[cam_key] is not None:
            img = observation[cam_key]
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow(cam_key, img)
            displayed_any = True
    if not displayed_any:
        for cam_key in observation:
            if isinstance(observation[cam_key], np.ndarray) and observation[cam_key].ndim == 3:
                img = observation[cam_key]
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.imshow(cam_key, img)
                break
    cv2.waitKey(1)

    # 1. Check ROS 2 /cmd_vel
    base_action = robot._from_twist_to_base_action()

    # 2. Check direct keyboard inputs if no ROS 2 command was sent
    if not base_action:
        keys = keyboard.get_action()
        if keys:
            kb_action = robot._from_keyboard_to_base_action(keys)
            if kb_action and (kb_action["x.vel"] != 0.0 or kb_action["y.vel"] != 0.0 or kb_action["theta.vel"] != 0.0):
                base_action = kb_action

    if base_action:
        # Send base action to robot (does not re-enable or command arm motors)
        _ = robot.send_action(base_action)

    precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
