#!/usr/bin/env python3

import time
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.feetech import TorqueMode
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.scripts.control_robot import busy_wait
from lerobot.common.robot_devices.motors.csvmotors import CsvMotorsBus


def main():

    follower_arm = FeetechMotorsBus(
        port="/dev/ttyUSB0",
        motors={
            # name: (index, model)
            "shoulder_pan": (1, "sts3215"),
            "shoulder_lift": (2, "sts3215"),
            "elbow_flex": (3, "sts3215"),
            "wrist_flex": (4, "sts3215"),
            "wrist_roll": (5, "sts3215"),
            "gripper": (6, "sts3215"),
        },
    )

    leader_arm = CsvMotorsBus(
        port="outputs/visualize_dataset_html/astroyat/latest/static/",
        motors={
            # name: (index, model)
            "shoulder_pan": (1, "csv"),
            "shoulder_lift": (2, "csv"),
            "elbow_flex": (3, "csv"),
            "wrist_flex": (4, "csv"),
            "wrist_roll": (5, "csv"),
            "gripper": (6, "csv"),
        },
    )

    robot = ManipulatorRobot(
        robot_type="so100",
        leader_arms={"main": leader_arm},
        follower_arms={"main": follower_arm},
        calibration_dir=".cache/calibration/so100",
    )

    robot.connect()
    robot.follower_arms["main"].write("Torque_Enable", TorqueMode.ENABLED.value)

    for i in range(5):
        robot.leader_arms["main"].prerecord_episode(i)
        fps = 30
        lines = robot.leader_arms["main"].lines
        for _ in range(lines):
            start_time = time.perf_counter()
            robot.teleop_step()
            dt_s = time.perf_counter() - start_time
            busy_wait(1 / fps - dt_s)

    robot.disconnect()


if __name__ == "__main__":
    main()
