#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

import csv
import logging
import time
from io import StringIO
from pathlib import Path

from lerobot.processor import RobotAction

from ..teleoperator import Teleoperator
from .config_csvarm_leader import CsvArmLeaderConfig
from .grabber import Grabber

logger = logging.getLogger(__name__)


class CsvArmLeader(Teleoperator):
    """Teleoperator that reads actions from a CSV file."""

    config_class = CsvArmLeaderConfig
    name = "csvarm_leader"

    def __init__(self, config: CsvArmLeaderConfig):
        super().__init__(config)
        self.config = config

        self.csvaction = CsvAction()
        self.lines = 0
        self.grabber = Grabber(self.csvaction)
        self.grabber.teleop_arm = self
        self._is_connected = False

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "shoulder_pan.pos": float,
            "shoulder_lift.pos": float,
            "elbow_flex.pos": float,
            "wrist_flex.pos": float,
            "wrist_roll.pos": float,
            "gripper.pos": float,
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if Path(self.config.port).exists():
            self.csvaction.reset()
            with open(self.config.port, "r") as f:
                self.csvaction.fp.write(f.read())
            self.lines = self.csvaction.open_reader()
        self._is_connected = True
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> RobotAction:
        start = time.perf_counter()

        if self.lines > 0:
            try:
                row = next(self.csvaction.reader)
                row2 = list(map(float, row))
                if len(row2) > 6:
                    joint = row2[7:13]
                else:
                    joint = row2
                self.grabber.set_joint(joint)
                self.lines = self.lines - 1
            except (StopIteration, ValueError):
                joint = self.grabber.joint
                self.lines = 0
        else:
            joint = self.grabber.joint

        action = {
            "shoulder_pan.pos": float(joint[0]),
            "shoulder_lift.pos": float(joint[1]),
            "elbow_flex.pos": float(joint[2]),
            "wrist_flex.pos": float(joint[3]),
            "wrist_roll.pos": float(joint[4]),
            "gripper.pos": float(joint[5]),
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        self._is_connected = False
        logger.info(f"{self} disconnected.")


class CsvAction:
    def __init__(self):
        self.fp = StringIO()
        self.writer = None
        self.reader = None

    def reset(self):
        self.fp.seek(0)
        self.fp.truncate(0)

    def open_writer(self):
        self.reset()
        self.writer = csv.writer(self.fp)

    def open_reader(self):
        self.fp.seek(0)
        self.reader = csv.reader(self.fp)
        lines = sum(1 for _ in self.reader)
        self.fp.seek(0)
        self.reader = csv.reader(self.fp)
        return lines
