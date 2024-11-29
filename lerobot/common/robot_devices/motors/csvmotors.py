import time
import numpy as np

from lerobot.common.robot_devices.utils import (
    RobotDeviceAlreadyConnectedError,
    RobotDeviceNotConnectedError,
)
from lerobot.common.utils.utils import capture_timestamp_utc
from lerobot.scripts.control_robot import busy_wait
from lerobot.common.robot_devices.motors.csvaction import Csvaction


class CsvMotorsBus:
    def __init__(
        self,
        port,
        motors: dict[str, tuple[int, str]],
    ):
        self.port = port
        self.motors = motors
        self.is_connected = False

        self.csvaction = Csvaction(port)
        self.lines = 0
        self.grabber = None

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                f"CsvmotorMotorsBus({self.port}) is already connected. Do not call `motors_bus.connect()` twice."
            )
        self.is_connected = True

    def set_grabber(self, grabber):
        self.grabber = grabber
        if self.grabber is not None:
            self.grabber.csvaction = self.csvaction

    def prerecord_episode(self, episode_index):
        if self.grabber is not None:
            self.grabber.prerecord_episode()
        else:
            self.csvaction.csv_filename = self.port + "episode_" + str(episode_index) + ".csv"
        self.lines = self.csvaction.open_reader()

    def reconnect(self):
        self.is_connected = True

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    def set_calibration(self, calibration: dict[str, list]):
        pass

    def read(self, data_name, motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"CsvmotorMotorsBus({self.port}) is not connected. You need to run `motors_bus.connect()`."
            )

        start_time = time.perf_counter()

        if motor_names is None:
            motor_names = self.motor_names

        if isinstance(motor_names, str):
            motor_names = [motor_names]

        if self.lines > 0:
            row = next(self.csvaction.reader)
            row2 = list(map(float, row))
            if len(row2) > 6:
                pos = row2[7:13]
            else:
                pos = row2

            self.lines = self.lines - 1
            if self.lines == 0:
                self.csvaction.close()
        elif self.grabber is not None:
            pos = self.grabber.joint
        else:
            return None

        values = np.array(pos)
        return values

    def write(
        self,
        data_name,
        values: int | float | np.ndarray,
        motor_names: str | list[str] | None = None,
    ):
        pass

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"CsvmotorMotorsBus({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )

        self.is_connected = False
        self.lines = 0

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
