#!/usr/bin/env python

import logging
import time

from ..teleoperator import Teleoperator
from .config_csvarm_leader import CsvArmLeaderConfig

logger = logging.getLogger(__name__)


class CsvArmLeader(Teleoperator):
    """ """

    config_class = CsvArmLeaderConfig
    name = "csvarm_leader"

    def __init__(self, config: CsvArmLeaderConfig):
        super().__init__(config)
        self.config = config

        self.csvaction = CsvAction()
        self.lines = 0
        from grabber import Grabber

        self.grabber = Grabber(self.csvaction)
        self.grabber.teleop_arm = self

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return True

    def connect(self) -> None:
        pass

    @property
    def is_calibrated(self) -> bool:
        pass

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()

        if self.lines > 0:
            row = next(self.csvaction.reader)
            row2 = list(map(float, row))
            if len(row2) > 6:
                joint = row2[7:13]
            else:
                joint = row2
            self.grabber.set_joint(joint)
            self.lines = self.lines - 1
        else:
            joint = self.grabber.joint

        action = {
            "shoulder_pan": float(joint[0]),
            "shoulder_lift": float(joint[1]),
            "elbow_flex": float(joint[2]),
            "wrist_flex": float(joint[3]),
            "wrist_roll": float(joint[4]),
            "gripper": float(joint[5]),
        }
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        pass


import csv
from io import StringIO


class CsvAction:
    def __init__(self):
        self.fp = StringIO()
        self.writer = None
        self.reader = None

    def open_writer(self):
        self.fp.seek(0)
        self.fp.truncate(0)
        self.writer = csv.writer(self.fp)

    def open_reader(self):
        self.fp.seek(0)
        self.reader = csv.reader(self.fp)
        lines = sum(1 for _ in self.reader)
        self.fp.seek(0)
        return lines
