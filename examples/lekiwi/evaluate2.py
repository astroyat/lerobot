#!/usr/bin/env python

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

import argparse
import time
from pathlib import Path
import cv2
import torch

def show_wrist_frame(frame, title="Live Wrist View (Autonomous Policy)"):
    if frame is not None:
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.putText(bgr, title, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("image", bgr)
        cv2.waitKey(1)

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.configs import PreTrainedConfig
from lerobot.policies.factory import make_policy, make_pre_post_processors
import lerobot.policies.smolvla.configuration_smolvla
import lerobot.policies.g05.configuration_g05
from lerobot.policies.utils import make_robot_action
from lerobot.robots.lekiwi.config_lekiwi import LeKiwiClientConfig
from lerobot.robots.lekiwi.lekiwi_client import LeKiwiClient
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.csvarm_leader import CsvArmLeader, CsvArmLeaderConfig
from lerobot.utils.constants import ACTION, OBS_STR, HF_LEROBOT_HOME
from lerobot.utils.feature_utils import hw_to_dataset_features
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import log_say

NUM_EPISODES = 10
FPS = 30
EPISODE_TIME_SEC = 60  # Maximum seconds per autonomous episode (0 = unlimited / stop with right arrow)
RESET_TIME_SEC = 5
TASK_DESCRIPTION = "Pick white object"
TRAIN_REPO_ID = "astroyat/pickwhiteobject2"

# Default to SmolVLA checkpoint; fallback to 020000 or G0.5
DEFAULT_CHECKPOINT = Path("outputs/train/smolvla_lekiwi/checkpoints/last/pretrained_model")
if not DEFAULT_CHECKPOINT.exists():
    DEFAULT_CHECKPOINT = Path("outputs/train/smolvla_lekiwi/checkpoints/020000/pretrained_model")
if not DEFAULT_CHECKPOINT.exists():
    DEFAULT_CHECKPOINT = Path("outputs/train/g05_lekiwi/checkpoints/last/pretrained_model")

EVAL_REPO_ID = "astroyat/eval_pickwhiteobject2"
RECORD_EVAL_DATASET = False  # Set to True to record evaluation episodes into a dataset


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate autonomous policy on LeKiwi")
    parser.add_argument(
        "--checkpoint_path",
        "--checkpoint-path",
        "--policy.path",
        type=str,
        default=str(DEFAULT_CHECKPOINT),
        help="Path to pretrained/fine-tuned model directory (e.g. outputs/train/smolvla_lekiwi/checkpoints/last/pretrained_model)",
    )
    parser.add_argument(
        "--task",
        type=str,
        default=TASK_DESCRIPTION,
        help="Task description string for the VLA policy",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=NUM_EPISODES,
        help="Number of evaluation episodes",
    )
    parser.add_argument(
        "--time",
        type=int,
        default=EPISODE_TIME_SEC,
        help="Episode duration in seconds (0 = unlimited)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    checkpoint_path = Path(args.checkpoint_path).resolve()
    task_desc = args.task
    num_episodes = args.episodes
    episode_time = args.time

    policy_cfg = PreTrainedConfig.from_pretrained(str(checkpoint_path))
    policy_name = getattr(policy_cfg, "type", "policy").upper()
    print(f"\n=======================================================")
    print(f" [Policy] Active Model: {policy_name}")
    print(f" [Policy] Checkpoint:   {checkpoint_path}")
    print(f" [Policy] Task Prompt:  \"{task_desc}\"")
    print(f"=======================================================\n")

    train_dataset = LeRobotDataset(TRAIN_REPO_ID)
    policy = make_policy(policy_cfg, ds_meta=train_dataset.meta)
    policy.eval()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=checkpoint_path,
        dataset_stats=train_dataset.meta.stats,
    )
    print(f"[Policy] {policy_name} and pre/post-processors loaded successfully!\n")

    # Create the robot and teleoperator configurations
    robot_config = LeKiwiClientConfig(remote_ip="10.0.0.71", id="lekiwi")
    robot_config.cameras.pop("front", None)  # Use wrist camera only
    leader_arm_config = CsvArmLeaderConfig(port="arm.csv", id="csvarm_leader_arm")
    keyboard_config = KeyboardTeleopConfig()

    # Initialize the robot and teleoperator
    robot = LeKiwiClient(robot_config)
    leader_arm = CsvArmLeader(leader_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

    # Optional evaluation dataset
    dataset = None
    if RECORD_EVAL_DATASET:
        action_features = hw_to_dataset_features(robot.action_features, ACTION)
        obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
        dataset_features = {**action_features, **obs_features}
        eval_root = HF_LEROBOT_HOME / EVAL_REPO_ID
        if eval_root.exists():
            dataset = LeRobotDataset.resume(repo_id=EVAL_REPO_ID, root=eval_root, image_writer_threads=4)
        else:
            dataset = LeRobotDataset.create(
                repo_id=EVAL_REPO_ID,
                fps=FPS,
                features=dataset_features,
                robot_type=robot.name,
                use_videos=True,
                image_writer_threads=4,
            )

    try:
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("image", 640, 480)
        cv2.setWindowProperty("image", cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass

    # Connect to LeKiwi
    robot.connect()
    leader_arm.connect()
    keyboard.connect()

    leader_arm.grabber.robot = robot
    leader_arm.grabber.teleop_arm = leader_arm

    observation = robot.flush_frames(num_fresh_frames=5)
    if "wrist" in observation:
        show_wrist_frame(observation["wrist"], title="Connected to LeKiwi")
    leader_arm.grabber.set_joint(observation["observation.state"][:6])

    listener, events = init_keyboard_listener()

    if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    base_action = {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }

    # Move arm to initial rest pose
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

    print("\n[Evaluate] Ready! Entering main autonomous evaluation loop.")
    print("  Controls:")
    print("    -> RIGHT ARROW: finish current episode")
    print("    <- LEFT ARROW:  discard current episode")
    print("    ESC:            emergency stop / exit")

    recorded_episodes = 0
    while recorded_episodes < num_episodes and not events["stop_recording"]:
        print(f"\n==========================================")
        print(f" Evaluation Episode {recorded_episodes + 1}/{num_episodes} [{policy_name}] ")
        print(f"==========================================")
        log_say(f"Running inference episode {recorded_episodes + 1}")

        # Move arm to camera posture before starting autonomous rollout
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

        # Clear in-transit frames so policy uses a fresh stationary image
        print("[Camera] Arm at camera posture. Purging in-transit frames...")
        fresh_obs = robot.flush_frames(num_fresh_frames=5)
        if "wrist" in fresh_obs and fresh_obs["wrist"] is not None:
            show_wrist_frame(fresh_obs["wrist"], title="Camera Posture Ready")
        policy.reset()

        print(f"[Policy] Autonomous {policy_name} policy taking control of LeKiwi...")
        t_start = time.perf_counter()
        step_count = 0
        events["exit_early"] = False

        while not events["exit_early"] and not events["stop_recording"]:
            t0 = time.perf_counter()
            if episode_time > 0 and (t0 - t_start) >= episode_time:
                print(f"[Policy] Episode time limit reached ({episode_time}s).")
                break

            obs = robot.get_observation()
            if "wrist" in obs and obs["wrist"] is not None:
                show_wrist_frame(obs["wrist"], title=f"Autonomous {policy_name} Policy [Active]")

            if "observation.state" not in obs or "wrist" not in obs or obs["wrist"] is None:
                precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
                continue

            # Format observation tensor for policy preprocessor
            raw_obs = {
                "observation.state": torch.from_numpy(obs["observation.state"]).float().unsqueeze(0),
                "observation.images.wrist": torch.from_numpy(obs["wrist"]).permute(2, 0, 1).float().unsqueeze(0) / 255.0,
                "task": task_desc,
            }

            model_obs = preprocessor(raw_obs)

            with torch.no_grad():
                out = policy.select_action(model_obs)
                action_tensor = out["action"] if isinstance(out, dict) else out

            processed_action = postprocessor(action_tensor)
            robot_action = make_robot_action(processed_action, train_dataset.features)

            # Send action directly to LeKiwi (arm positions + base velocities)
            robot.send_action(robot_action)

            if dataset is not None:
                dataset.add_frame({**robot_action, **obs})

            step_count += 1
            if step_count % 30 == 0:
                print(
                    f"  Step #{step_count} (vel: x={robot_action.get('x.vel', 0.0):.2f}, "
                    f"theta={robot_action.get('theta.vel', 0.0):.2f})",
                    flush=True,
                )

            precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

        # Stop base movement
        robot.send_action(base_action)

        if events["rerecord_episode"]:
            print("[Evaluate] Episode discarded, repeating...")
            events["rerecord_episode"] = False
            events["exit_early"] = False
            if dataset is not None:
                dataset.clear_episode_buffer()
            print("[Evaluate] Purging in-transit frames before repeating...")
            robot.flush_frames(num_fresh_frames=5)
            continue

        if dataset is not None:
            dataset.save_episode()
            print(f"[Save] Eval episode {recorded_episodes + 1} saved!")

        recorded_episodes += 1

        if not events["stop_recording"] and recorded_episodes < num_episodes:
            print(f"\n[Reset] Resetting environment for {RESET_TIME_SEC} seconds (place object)...")
            log_say("Reset the environment")
            t_reset_start = time.perf_counter()
            while (time.perf_counter() - t_reset_start) < RESET_TIME_SEC and not events["stop_recording"]:
                t0 = time.perf_counter()
                remaining = RESET_TIME_SEC - (t0 - t_reset_start)
                robot.send_action(base_action)
                obs = robot.get_observation()
                if "wrist" in obs and obs["wrist"] is not None:
                    show_wrist_frame(
                        obs["wrist"],
                        title=f"Resetting Environment - Place Object ({max(0.0, remaining):.1f}s)",
                    )
                precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

            # Flush any frames accumulated during the reset period
            print("[Reset] Purging in-transit frames after reset...")
            fresh_obs = robot.flush_frames(num_fresh_frames=5)
            if "wrist" in fresh_obs and fresh_obs["wrist"] is not None:
                show_wrist_frame(fresh_obs["wrist"], title="Reset Complete")

    # Clean up
    print("\n[Finished] Stopping evaluation...")
    log_say("Stop evaluation")
    if robot.is_connected:
        robot.disconnect()
    if leader_arm.is_connected:
        leader_arm.disconnect()
    if keyboard.is_connected:
        keyboard.disconnect()
    listener.stop()
    cv2.destroyAllWindows()

    if dataset is not None:
        dataset.finalize()
        try:
            dataset.push_to_hub()
        except Exception:
            pass


if __name__ == "__main__":
    main()
