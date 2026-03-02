#!/usr/bin/env python3

import logging
import time

import numpy as np

from lerobot.utils.robot_utils import precise_sleep

try:
    from ikmove import Ikmove
except ImportError:
    Ikmove = None

try:
    from detector import Detector
except ImportError:
    Detector = None

logger = logging.getLogger(__name__)

np.set_printoptions(suppress=True)
np.set_printoptions(precision=5)


class Grabber:
    def __init__(self, csvaction):

        self.middle_deg = [0, 0, 0, 0, 0, 0]
        self.reset_deg = [0, -90, 0, 90, 0, 0]
        self.up_deg = [0, 0, -90, 0, 0, 0]
        self.rotate_left_deg = [90, 0, 0, 90, 0, 80]
        self.rotate_right_deg = [-90, 0, 0, 90, 0, 80]

        self.camera_open_deg = [0, 0, 0, 45, 0, 80]
        self.camera_close_deg = [0, 0, 0, 45, 0, 0]
        self.grab_open_deg = [0, 50, 20, -30, 0, 80]
        self.grab_close_deg = [0, 50, 20, -30, 0, 0]

        if Ikmove is None:
            logger.warning("ikmove library not found. Inverse kinematics features will be disabled.")
            self.ikmove = None
            self.pos = None
            self.end_fk = None
            self.end_pos = None
        else:
            self.ikmove = Ikmove()
            self.joint = np.array(self.reset_deg)
            _, self.pos = self.ikmove.joint_to_pos(self.joint)
            self.end_fk = self.pos[7]
            self.end_pos = self.end_fk[:3, 3]

        self.joint = np.array(self.reset_deg)
        self.detector = None
        self.csvaction = csvaction

        self.robot = None
        self.teleop_arm = None

        self.is_detect = False
        self.is_base = False
        self.is_base2 = False
        self.is_policy = False

        self.detect_x = -1.0
        self.count_x = 0

        self.robot_description: str = """
Follow these instructions precisely. Never deviate.

You control a 3D printed robot with 6 DOF arm including a gripper mounted on top of a 3 wheel holonomic drive base.
The drive base can move forward, backward, right or left by 1 cm each time.
"""

    def set_joint(self, joint):
        self.joint = joint
        if self.ikmove is not None:
            _, self.pos = self.ikmove.joint_to_pos(self.joint)
            self.end_fk = self.pos[7]
            self.end_pos = self.end_fk[:3, 3]

    def count_joint_times(self, joints, speed):
        n = len(joints) - 1
        max_vals = np.zeros(n)
        for i in range(n):
            q0 = joints[i]
            qT = joints[i + 1]
            for j in range(len(q0)):
                diff = abs(q0[j] - qT[j])
                if j < 4:
                    diff = diff * speed
                if diff > max_vals[i]:
                    max_vals[i] = diff
        return max_vals

    def move_joints(self, joints, speed=1, replay=False, fps=30):
        if self.ikmove is None:
            logger.error("ikmove library not found. Cannot move joints.")
            return 0

        trajectory = self.ikmove.joint_trajectory(
            joints, self.count_joint_times(joints, speed=speed)
        )
        self.csvaction.open_writer()
        for joint in trajectory["joint_positions"]:
            if joint[4] < -90:
                joint[4] = -90
            elif joint[4] > 90:
                joint[4] = 90
            if joint[5] < 0:
                joint[5] = 0
            elif joint[5] > 80:
                joint[5] = 80
            joint_float = [float(x) for x in joint]
            if self.csvaction.writer is not None:
                self.csvaction.writer.writerow(joint_float)
        lines = len(trajectory["time_points"])
        if replay is False:
            self.teleop_arm.lines = self.csvaction.open_reader()
        else:
            self.replay(fps=fps)
        return lines

    def move_pos(self, pos, speed=1, replay=False, fps=30):
        if self.ikmove is None:
            logger.error("ikmove library not found. Cannot move to position.")
            return

        _, joint = self.ikmove.pos_to_joint(pos)
        joint[4] = self.joint[4]
        joint[5] = self.joint[5]
        self.move_joints([self.joint, joint], speed=speed, replay=replay, fps=fps)

    def move_pos_right(self):
        if self.end_pos is not None:
            self.end_pos[0] -= 0.01
            self.move_pos(self.end_pos)

    def move_pos_left(self):
        if self.end_pos is not None:
            self.end_pos[0] += 0.01
            self.move_pos(self.end_pos)

    def move_pos_forward(self):
        if self.end_pos is not None:
            self.end_pos[1] -= 0.01
            self.move_pos(self.end_pos)

    def move_pos_backward(self):
        if self.end_pos is not None:
            self.end_pos[1] += 0.01
            self.move_pos(self.end_pos)

    def move_pos_down(self):
        if self.end_pos is not None:
            self.end_pos[2] -= 0.01
            self.move_pos(self.end_pos)

    def move_pos_up(self):
        if self.end_pos is not None:
            self.end_pos[2] += 0.01
            self.move_pos(self.end_pos)

    def rotate_right(self):
        self.joint[4] -= 1
        return self.move_joints([self.joint])

    def rotate_left(self):
        self.joint[4] += 1
        return self.move_joints([self.joint])

    def rotate_open(self):
        self.joint[5] -= 1
        return self.move_joints([self.joint])

    def rotate_close(self):
        self.joint[5] += 1
        return self.move_joints([self.joint])

    def move_init(self):
        return self.move_joints([self.reset_deg])

    def move_keyboard(self, keyboard_keys):
        if "k" in keyboard_keys:
            self.move_init()
        elif "i" in keyboard_keys:
            self.move_pos_forward()
        elif "," in keyboard_keys:
            self.move_pos_backward()
        elif "j" in keyboard_keys:
            self.move_pos_right()
        elif "l" in keyboard_keys:
            self.move_pos_left()
        elif "u" in keyboard_keys:
            self.move_pos_up()
        elif "m" in keyboard_keys:
            self.move_pos_down()
        elif "o" in keyboard_keys:
            self.rotate_right()
        elif "p" in keyboard_keys:
            self.rotate_left()
        elif "." in keyboard_keys:
            self.rotate_open()
        elif "/" in keyboard_keys:
            self.rotate_close()

    def move_calib(self):
        joints = []
        joints.append(self.joint)
        # joints.append(self.middle_deg)
        joints.append(self.reset_deg)
        # joints.append(self.up_deg)
        # joints.append(self.rotate_left_deg)
        # joints.append(self.rotate_right_deg)
        # joints.append(self.camera_close_deg)
        return self.move_joints(joints)

    def move_camera(self):
        joints = []
        joints.append(self.joint)
        joints.append(self.camera_open_deg)
        return self.move_joints(joints)

    def move_grab(self):
        joints = []
        joints.append(self.joint)
        joints.append(self.grab_open_deg)
        joints.append(self.grab_close_deg)
        joints.append(self.camera_close_deg)
        joints.append(self.camera_open_deg)
        return self.move_joints(joints)

    def get_img(self):
        observation = self.robot.get_observation()
        return observation["camera1"]

    def record_episode_detect(self):
        # prompt = "detect white object"
        base_action = {
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }

        if Detector is None:
            logger.error("detector library not found. Cannot record episode detect.")
            return base_action

        if self.detector is None:
            self.detector = Detector()

        # x1, y1, x2, y2 = self.detector.detect_object_paligemma(
        x1, y1, x2, y2 = self.detector.detect_object_color(
            0,
            0,
            self.detector.width,
            self.detector.height - 80,
            img=self.get_img(),
            # prompt=prompt,
        )
        if x1 == 0 and y1 == 0 and x2 == 0 and y2 == 0:
            if self.count_x > 5:
                self.detect_x = -1 * self.detect_x
                self.count_x = 0
            else:
                self.count_x += 1
            base_action["x.vel"] = self.detect_x
        else:
            self.is_detect = False
            self.is_base = True

        return base_action

    def record_episode_base(self):
        target_x = 320
        target_y = 240
        target_color = (0, 255, 0)
        base_action = {
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }

        if Detector is None:
            logger.error("detector library not found. Cannot record episode base.")
            return base_action

        if self.detector is None:
            self.detector = Detector()

        x, y = self.detector.track_object(
            target_x, target_y, target_color, img=self.get_img()
        )

        if x == 0 and y == 0:
            return base_action

        if self.is_base is True:
            diff = 5
            if abs(x - target_x) < diff and abs(y - target_y) < diff:
                self.is_base = False
                if self.is_base2 is False:
                    self.is_base2 = True
                    self.is_detect = True
                else:
                    self.is_detect = False
                    self.is_base = False
                    self.is_base2 = False
                    self.move_grab()
            else:
                base_action["theta.vel"] = (target_x - x) / 100
                base_action["x.vel"] = (target_y - y) / 5000

        return base_action

    def replay(self, fps=30):
        lines = self.csvaction.open_reader()
        for _ in range(lines):
            start_time = time.perf_counter()

            row = next(self.csvaction.reader)
            row2 = list(map(float, row))
            if len(row2) > 6:
                joint = row2[7:13]
            else:
                joint = row2
            self.joint = joint

            dt_s = time.perf_counter() - start_time
            precise_sleep(max(1.0 / fps - dt_s, 0.0))

        if self.ikmove is not None:
            _, self.pos = self.ikmove.joint_to_pos(self.joint)
            self.end_fk = self.pos[7]
            self.end_pos = self.end_fk[:3, 3]

    def close(self):
        if self.detector is not None:
            self.detector.close()
            self.detector = None
