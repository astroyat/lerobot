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

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
#from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.csvarm_leader import CsvArmLeader, CsvArmLeaderConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
#from lerobot.utils.visualization_utils import init_rerun

NUM_EPISODES = 2
FPS = 30
EPISODE_TIME_SEC = 0
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "Pick paper ball"
#HF_MODEL_ID = "astroyat/act_paperball24"
HF_MODEL_ID = "astroyat/smolvla_paperball24"
HF_REPO_ID = "astroyat/eval_paperball2"

# Create the robot and teleoperator configurations
robot_config = LeKiwiClientConfig(remote_ip="192.168.68.56", id="lekiwi")
leader_arm_config = CsvArmLeaderConfig(port="arm.csv", id="csvarm_leader_arm")
keyboard_config = KeyboardTeleopConfig()

# Initialize the robot and teleoperator
robot = LeKiwiClient(robot_config)
leader_arm = CsvArmLeader(leader_arm_config)
keyboard = KeyboardTeleop(keyboard_config)

# Create policy
#policy = ACTPolicy.from_pretrained(HF_MODEL_ID)
policy = SmolVLAPolicy.from_pretrained(HF_MODEL_ID)

# TODO(Steven): Update this example to use pipelines
teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

# Configure the dataset features
action_features = hw_to_dataset_features(robot.action_features, ACTION)
obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
dataset_features = {**action_features, **obs_features}

# Create the dataset
dataset = LeRobotDataset.create(
    repo_id=HF_REPO_ID,
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
)

# Build Policy Processors
preprocessor, postprocessor = make_pre_post_processors(
    policy_cfg=policy,
    pretrained_path=HF_MODEL_ID,
    dataset_stats=dataset.meta.stats,
    # The inference device is automatically set to match the detected hardware, overriding any previous device settings from training to ensure compatibility.
    preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
)

# Connect the robot and teleoperator
# To connect you already should have this script running on LeKiwi: `python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi`
robot.connect()
leader_arm.connect()
keyboard.connect()

leader_arm.grabber.robot = robot
leader_arm.grabber.teleop_arm = leader_arm

observation = robot.get_observation()
leader_arm.grabber.set_joint(observation["observation.state"][:6])

# Initialize the keyboard listener and rerun visualization
listener, events = init_keyboard_listener()
#init_rerun(session_name="lekiwi_evaluate")

if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
    raise ValueError("Robot or teleop is not connected!")

print("Starting evaluate loop...")
recorded_episodes = 0
while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
    log_say(f"Running inference, recording eval episode {recorded_episodes} of {NUM_EPISODES}")

    leader_arm.grabber.is_policy = True

    # Main record loop
    record_loop(
        robot=robot,
        events=events,
        fps=FPS,
        policy=policy,
        preprocessor=preprocessor,  # Pass the pre and post policy processors
        postprocessor=postprocessor,
        dataset=dataset,
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK_DESCRIPTION,
        display_data=False,
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
    )

    # Reset the environment if not stopping or re-recording
    if not events["stop_recording"] and (
        (recorded_episodes < NUM_EPISODES - 1) or events["rerecord_episode"]
    ):
        log_say("Reset the environment")
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            control_time_s=RESET_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=False,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
        )

    if events["rerecord_episode"]:
        log_say("Re-record episode")
        events["rerecord_episode"] = False
        events["exit_early"] = False
        dataset.clear_episode_buffer()
        continue

    # Save episode
    dataset.save_episode()
    recorded_episodes += 1

# Clean up
log_say("Stop recording")
robot.disconnect()
leader_arm.disconnect()
keyboard.disconnect()
listener.stop()

dataset.finalize()
dataset.push_to_hub()
