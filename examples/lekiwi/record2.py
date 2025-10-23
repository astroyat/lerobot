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

def show_wrist_frame(frame, title="Live Wrist View"):
    if frame is not None:
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.putText(bgr, title, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("image", bgr)
        cv2.waitKey(1)

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi.config_lekiwi import LeKiwiClientConfig
from lerobot.robots.lekiwi.lekiwi_client import LeKiwiClient
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.csvarm_leader import CsvArmLeader, CsvArmLeaderConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import log_say

NUM_EPISODES = 50
FPS = 30
EPISODE_TIME_SEC = 0
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "Pick white object"
HF_REPO_ID = "astroyat/pickwhiteobject2"
AUTO_DETECT = True  # Set to True for autonomous visual search, False for direct teleop recording

def main():
    # Create the robot and teleoperator configurations
    robot_config = LeKiwiClientConfig(remote_ip="10.0.0.71", id="lekiwi")
    leader_arm_config = CsvArmLeaderConfig(port="arm.csv", id="csvarm_leader_arm")
    keyboard_config = KeyboardTeleopConfig()

    # Initialize the robot and teleoperator
    robot = LeKiwiClient(robot_config)
    leader_arm = CsvArmLeader(leader_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

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

    # Connect the robot and teleoperator
    # To connect you already should have this script running on LeKiwi: `python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi`
    robot.connect()
    leader_arm.connect()
    keyboard.connect()

    leader_arm.grabber.robot = robot
    leader_arm.grabber.teleop_arm = leader_arm

    try:
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("image", 640, 480)
        cv2.setWindowProperty("image", cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass

    def live_observation_processor(obs):
        res = robot_observation_processor(obs)
        if hasattr(leader_arm, "grabber") and leader_arm.grabber.is_base is True:
            pass
        elif "wrist" in obs and obs["wrist"] is not None:
            show_wrist_frame(obs["wrist"], title="Live Wrist View (Recording)")
        return res

    print("Flushing observation buffer to purge stale frames from previous run...")
    observation = robot.flush_frames(num_fresh_frames=5)
    if "wrist" in observation:
        show_wrist_frame(observation["wrist"], title="Connected to LeKiwi (Live Feed)")
    leader_arm.grabber.set_joint(observation["observation.state"][:6])

    # Initialize the keyboard listener and rerun visualization
    listener, events = init_keyboard_listener()

    if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    base_action = {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }

    print("Moving arm to initial rest pose...")
    lines = leader_arm.grabber.move_init()
    for _ in range(lines):
        t0 = time.perf_counter()
        arm_action = leader_arm.get_action()
        arm_action = {f"arm_{k}": v for k, v in arm_action.items()}
        robot.send_action(arm_action | base_action)
        obs = robot.get_observation()
        if "wrist" in obs:
            show_wrist_frame(obs["wrist"], title="Arm moving to rest pose")
        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

    print("\n[Record] Ready! Entering main episode loop.")
    recorded_episodes = 0
    while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
        print(f"\n==========================================")
        print(f" Episode {recorded_episodes + 1}/{NUM_EPISODES} ")
        print(f"==========================================")
        log_say(f"Recording episode {recorded_episodes}")

        leader_arm.grabber.reset_detect()

        print("[Camera] Moving arm to camera posture...")
        lines = leader_arm.grabber.move_camera()
        for _ in range(lines):
            t0 = time.perf_counter()
            arm_action = leader_arm.get_action()
            arm_action = {f"arm_{k}": v for k, v in arm_action.items()}
            robot.send_action(arm_action | base_action)
            obs = robot.get_observation()
            if "wrist" in obs:
                show_wrist_frame(obs["wrist"], title="Arm moving to camera posture")
            precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

        # Clear in-transit frames so detector uses a fresh stationary image
        print("[Camera] Arm at camera posture. Purging in-transit frames...")
        fresh_obs = robot.flush_frames(num_fresh_frames=5)
        if "wrist" in fresh_obs and fresh_obs["wrist"] is not None:
            show_wrist_frame(fresh_obs["wrist"], title="Camera Posture Ready")

        if AUTO_DETECT:
            print("[Detect] Searching for target object in camera view...")
            leader_arm.grabber.is_detect = True
            detect_count = 0
            while leader_arm.grabber.is_detect is True and not events["stop_recording"]:
                t0 = time.perf_counter()
                arm_action = leader_arm.get_action()
                arm_action = {f"arm_{k}": v for k, v in arm_action.items()}
                base_action = leader_arm.grabber.record_episode_detect()
                robot.send_action(arm_action | base_action)
                detect_count += 1
                if detect_count == 1 or detect_count % 15 == 0:
                    print(f"  ... frame #{detect_count} searching (base x.vel: {base_action.get('x.vel', 0.0):.2f})", flush=True)
                precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
            if events["stop_recording"]:
                break
            print("[Record] Target detected! Recording started.")
        else:
            leader_arm.grabber.is_detect = False
            leader_arm.grabber.is_base = False
            print("[Record] Recording started.")

        print("  Controls:")
        print("    -> RIGHT ARROW: finish episode and save")
        print("    <- LEFT ARROW:  discard and re-record episode")
        print("    ESC:            stop session and save dataset")

        # Main record loop
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            dataset=dataset,
            teleop=[leader_arm, keyboard],
            control_time_s=EPISODE_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=False,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=live_observation_processor,
        )

        # Reset the environment if not stopping or re-recording
        if not events["stop_recording"] and (
            (recorded_episodes < NUM_EPISODES - 1) or events["rerecord_episode"]
        ):
            print("\n[Reset] Resetting environment for 10 seconds...")
            log_say("Reset the environment")
            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                teleop=[leader_arm, keyboard],
                control_time_s=RESET_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=False,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=live_observation_processor,
            )

        if events["rerecord_episode"]:
            print("[Re-record] Discarding episode, repeating...")
            log_say("Re-record episode")
            events["rerecord_episode"] = False
            events["exit_early"] = False
            dataset.clear_episode_buffer()
            continue

        # Save episode
        print(f"[Save] Episode {recorded_episodes + 1} saved to dataset!")
        dataset.save_episode()
        recorded_episodes += 1

    # Clean up
    print("\n[Finished] Stopping recording and finalizing dataset...")
    log_say("Stop recording")
    robot.disconnect()
    leader_arm.disconnect()
    keyboard.disconnect()
    listener.stop()
    cv2.destroyAllWindows()

    dataset.finalize()
    dataset.push_to_hub()


if __name__ == "__main__":
    main()
