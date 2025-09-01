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

import logging
import time
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceNotConnectedError
from lerobot.model.kinematics import RobotKinematics
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from .so101_follower import SO101Follower
from .config_so101_follower import SO101FollowerEndEffectorConfig

logger = logging.getLogger(__name__)


class SO101FollowerEndEffector(SO101Follower):
    """
    SO101Follower robot with end-effector space control.

    Inherits joint-space behavior from SO101Follower and adds forward/inverse
    kinematics to convert end-effector deltas to joint positions.
    """

    config_class = SO101FollowerEndEffectorConfig
    name = "so101_follower_end_effector"

    def __init__(self, config: SO101FollowerEndEffectorConfig):
        super().__init__(config)

        # Override bus to ensure degree-based normalization for EE control
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )

        self.cameras = make_cameras_from_configs(config.cameras)

        if self.config.urdf_path is None:
            raise ValueError(
                "urdf_path must be provided in the configuration for end-effector control."
            )

        self.kinematics = RobotKinematics(
            urdf_path=self.config.urdf_path,
            target_frame_name=self.config.target_frame_name,
        )

        self.end_effector_bounds = self.config.end_effector_bounds

        self.current_ee_pos = None
        self.current_joint_pos = None

    @property
    def action_features(self) -> dict[str, Any]:
        return {
            "dtype": "float32",
            "shape": (4,),
            "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "gripper": 3},
        }

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        ###########################################################
        # Debugging
        ###########################################################
        print(f"~~~~~~~~~~debug send_action for so_101_follower_end_effector {action}")
        # Check if this is a joint space action (from teleoperator)
        if isinstance(action, dict) and any(key.endswith('.pos') for key in action.keys()):
            # This is a joint space action, pass it directly to parent class
            return super().send_action(action)
        ###########################################################

        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Convert dict action with deltas to numpy array
        if isinstance(action, dict):
            if all(k in action for k in ["delta_x", "delta_y", "delta_z"]):
                delta_ee = np.array(
                    [
                        action["delta_x"] * self.config.end_effector_step_sizes["x"],
                        action["delta_y"] * self.config.end_effector_step_sizes["y"],
                        action["delta_z"] * self.config.end_effector_step_sizes["z"],
                    ],
                    dtype=np.float32,
                )
                if "gripper" not in action:
                    action["gripper"] = [1.0]
                action = np.append(delta_ee, action["gripper"])
            else:
                logger.warning(
                    f"Expected action keys 'delta_x', 'delta_y', 'delta_z', got {list(action.keys())}"
                )
                action = np.zeros(4, dtype=np.float32)

        if self.current_joint_pos is None:
            present = self.bus.sync_read("Present_Position")
            self.current_joint_pos = np.array([float(present[name]) for name in present], dtype=np.float64)

        if self.current_ee_pos is None:
            self.current_ee_pos = self.kinematics.forward_kinematics(self.current_joint_pos)

        desired_ee_pos = np.eye(4)
        desired_ee_pos[:3, :3] = self.current_ee_pos[:3, :3]
        desired_ee_pos[:3, 3] = self.current_ee_pos[:3, 3] + action[:3]

        if self.end_effector_bounds is not None:
            desired_ee_pos[:3, 3] = np.clip(
                desired_ee_pos[:3, 3], self.end_effector_bounds["min"], self.end_effector_bounds["max"]
            )

        target_joint_values_in_degrees = self.kinematics.inverse_kinematics(
            self.current_joint_pos, desired_ee_pos
        )

        joint_action = {
            f"{key}.pos": target_joint_values_in_degrees[i] for i, key in enumerate(self.bus.motors.keys())
        }

        # Gripper handling
        joint_action["gripper.pos"] = np.clip(
            self.current_joint_pos[-1] + (action[-1] - 1) * self.config.max_gripper_pos,
            5,
            self.config.max_gripper_pos,
        )

        self.current_ee_pos = desired_ee_pos.copy()
        self.current_joint_pos = target_joint_values_in_degrees.copy()
        self.current_joint_pos[-1] = joint_action["gripper.pos"]

        return super().send_action(joint_action)

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        obs_dict = self.bus.sync_read("Present_Position")
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        _ = (time.perf_counter() - start) * 1e3

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def reset(self):
        self.current_ee_pos = None
        self.current_joint_pos = None
