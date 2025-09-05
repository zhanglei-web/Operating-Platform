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

import abc
from dataclasses import dataclass, field
from typing import Sequence

import draccus

from operating_platform.robot.robots.com_configs.cameras import (
    CameraConfig,
    IntelRealSenseCameraConfig,
    OpenCVCameraConfig,
)

from operating_platform.robot.robots.com_configs.motors import (
    PikaMotorsBusConfig,
    PiperMotorsBusConfig,
    DynamixelMotorsBusConfig,
    FeetechMotorsBusConfig,
    MotorsBusConfig,
)


@dataclass
class RobotConfig(draccus.ChoiceRegistry, abc.ABC):
    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)


# TODO(rcadene, aliberts): remove ManipulatorRobotConfig abstraction
@dataclass
class ManipulatorRobotConfig(RobotConfig):
    leader_arms: dict[str, MotorsBusConfig] = field(default_factory=lambda: {})
    follower_arms: dict[str, MotorsBusConfig] = field(default_factory=lambda: {})
    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {})

    # Optionally limit the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length
    # as the number of motors in your follower arms (assumes all follower arms have the same number of
    # motors).
    max_relative_target: list[float] | float | None = None

    # Optionally set the leader arm in torque mode with the gripper motor set to this angle. This makes it
    # possible to squeeze the gripper and have it spring back to an open position on its own. If None, the
    # gripper is not put in torque mode.
    gripper_open_degree: float | None = None

    mock: bool = False

    def __post_init__(self):
        if self.mock:
            for arm in self.leader_arms.values():
                if not arm.mock:
                    arm.mock = True
            for arm in self.follower_arms.values():
                if not arm.mock:
                    arm.mock = True
            for cam in self.cameras.values():
                if not cam.mock:
                    cam.mock = True

        if self.max_relative_target is not None and isinstance(self.max_relative_target, Sequence):
            for name in self.follower_arms:
                if len(self.follower_arms[name].motors) != len(self.max_relative_target):
                    raise ValueError(
                        f"len(max_relative_target)={len(self.max_relative_target)} but the follower arm with name {name} has "
                        f"{len(self.follower_arms[name].motors)} motors. Please make sure that the "
                        f"`max_relative_target` list has as many parameters as there are motors per arm. "
                        "Note: This feature does not yet work with robots where different follower arms have "
                        "different numbers of motors."
                    )


@RobotConfig.register_subclass("aloha")
@dataclass
class AlohaRobotConfig(RobotConfig):
    start_pose = [-90.0, 90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
    joint_p_limit = [169.0, 102.0, 169.0, 52.0, 169.0, 117.0, 169.0]
    joint_n_limit = [-169.0, -102.0, -169.0, -167.0, -169.0, -87.0, -169.0]

    right_leader_arm = PiperMotorsBusConfig(
        port="can_right",
        motors={
            "joint_1": [1,  "piper-motor"],
            "joint_2": [2,  "piper-motor"],
            "joint_3": [3,  "piper-motor"],
            "joint_4": [4,  "piper-motor"],
            "joint_5": [5,  "piper-motor"],
            "joint_6": [6,  "piper-motor"],
            "pose_x":  [7,  "piper-pose"],
            "pose_y":  [8,  "piper-pose"],
            "pose_z":  [9,  "piper-pose"],
            "pose_rx": [10, "piper-pose"],
            "pose_ry": [11, "piper-pose"],
            "pose_rz": [12, "piper-pose"],
            "gripper": [13, "piper-gripper"],
        },
    )

    left_leader_arm = PiperMotorsBusConfig(
        port="can_left",
        motors={
            "joint_1": [1,  "piper-motor"],
            "joint_2": [2,  "piper-motor"],
            "joint_3": [3,  "piper-motor"],
            "joint_4": [4,  "piper-motor"],
            "joint_5": [5,  "piper-motor"],
            "joint_6": [6,  "piper-motor"],
            "pose_x":  [7,  "piper-pose"],
            "pose_y":  [8,  "piper-pose"],
            "pose_z":  [9,  "piper-pose"],
            "pose_rx": [10, "piper-pose"],
            "pose_ry": [11, "piper-pose"],
            "pose_rz": [12, "piper-pose"],
            "gripper": [13, "piper-gripper"],
        },
    )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "image_top": OpenCVCameraConfig(
                camera_index=1,
                fps=30,
                width=640,
                height=480,
            ),
            "image_right": OpenCVCameraConfig(
                camera_index=2,
                fps=30,
                width=640,
                height=480,
            ),
            "image_left": OpenCVCameraConfig(
                camera_index=3,
                fps=30,
                width=640,
                height=480,
            ),
            # "image_depth_top": OpenCVCameraConfig(
            #     camera_index=4,
            #     fps=30,
            #     width=640,
            #     height=400,
            # ),
            # "image_depth_right": OpenCVCameraConfig(
            #     camera_index=5,
            #     fps=30,
            #     width=640,
            #     height=400,
            # ),
            # "image_depth_left": OpenCVCameraConfig(
            #     camera_index=6,
            #     fps=30,
            #     width=640,
            #     height=400,
            # ),
        }
    )


@RobotConfig.register_subclass("adora")
@dataclass
class AdoraRobotConfig(RobotConfig):
    ip = "192.168.1.19"

    start_pose = [-90.0, 90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
    joint_p_limit = [169.0, 102.0, 169.0, 52.0, 169.0, 117.0, 169.0]
    joint_n_limit = [-169.0, -102.0, -169.0, -167.0, -169.0, -87.0, -169.0]

    leader_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": DynamixelMotorsBusConfig(
                port="/dev/ttyUSB0",
                motors={
                    "shoulder_pan": [1, "xl330-m288"],
                    "shoulder_lift": [2, "xl330-m288"],
                    "elbow_flex": [3, "xl330-m288"],
                    "wrist_flex": [4, "xl330-m288"],
                    "wrist_roll": [5, "xl330-m288"],
                    "wrist_1": [6, "xl330-m288"],
                    "weist_2": [7, "xl330-m288"],
                    "gripper": [8, "xl330-m288"],
                },
            ),
        }
    )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "top": OpenCVCameraConfig(
                camera_index=6,
                fps=30,
                width=640,
                height=480,
            ),
            "wrist": OpenCVCameraConfig(
                camera_index=14,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )


@RobotConfig.register_subclass("adora_dual")
@dataclass
class AdoraDualRobotConfig(RobotConfig):

    right_arm_config = {}
    right_arm_config['usb_port'] = "/dev/ttyUSB0"
    right_arm_config['ip'] = "192.168.1.19"
    right_arm_config['fps'] = 30
    right_arm_config['calibration_dir'] = ".cache/calibration/adora_dual_right"
    right_arm_config['start_pose'] = [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
    right_arm_config['joint_p_limit'] = [169.0, 102.0, 169.0, 52.0, 169.0, 117.0, 169.0]
    right_arm_config['joint_n_limit'] = [-169.0, -102.0, -169.0, -167.0, -169.0, -87.0, -169.0]
    right_leader_arm = DynamixelMotorsBusConfig(
        port=right_arm_config['usb_port'],
        motors={
            "joint_1": [1, "xl330-m288"],
            "joint_2": [2, "xl330-m288"],
            "joint_3": [3, "xl330-m288"],
            "joint_4": [4, "xl330-m288"],
            "joint_5": [5, "xl330-m288"],
            "joint_6": [6, "xl330-m288"],
            "joint_7": [7, "xl330-m288"],
            "gripper": [8, "xl330-m288"],
        },
    )

    left_arm_config = {}
    left_arm_config['usb_port'] = "/dev/ttyUSB1"
    left_arm_config['ip'] = "192.168.1.20"
    left_arm_config['fps'] = 30
    left_arm_config['calibration_dir'] = ".cache/calibration/adora_dual_left"
    left_arm_config['start_pose'] = [-90.0, 90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
    left_arm_config['joint_p_limit'] = [169.0, 102.0, 169.0, 52.0, 169.0, 117.0, 169.0]
    left_arm_config['joint_n_limit'] = [-169.0, -102.0, -169.0, -167.0, -169.0, -87.0, -169.0]
    left_leader_arm = DynamixelMotorsBusConfig(
        port=left_arm_config['usb_port'],
        motors={
            "joint_1": [1, "xl330-m288"],
            "joint_2": [2, "xl330-m288"],
            "joint_3": [3, "xl330-m288"],
            "joint_4": [4, "xl330-m288"],
            "joint_5": [5, "xl330-m288"],
            "joint_6": [6, "xl330-m288"],
            "joint_7": [7, "xl330-m288"],
            "gripper": [8, "xl330-m288"],
        },
    )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "top": OpenCVCameraConfig(
                camera_index=14,
                fps=30,
                width=640,
                height=480,
            ),
            "left_wrist": OpenCVCameraConfig(
                camera_index=6,
                fps=30,
                width=640,
                height=480,
            ),
            "right_wrist": OpenCVCameraConfig(
                camera_index=22,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    mock: bool = False


@RobotConfig.register_subclass("realman")
@dataclass
class RealmanRobotConfig(RobotConfig):

    right_arm_config = {}
    # right_arm_config['usb_port'] = "/dev/ttyUSB0"
    right_arm_config['ip'] = "169.254.128.19"
    right_arm_config['fps'] = 30
    # right_arm_config['calibration_dir'] = ".cache/calibration/adora_dual_right"
    # right_arm_config['start_pose'] = [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
    # right_arm_config['joint_p_limit'] = [169.0, 102.0, 169.0, 52.0, 169.0, 117.0, 169.0]
    # right_arm_config['joint_n_limit'] = [-169.0, -102.0, -169.0, -167.0, -169.0, -87.0, -169.0]
    # right_leader_arm = DynamixelMotorsBusConfig(
    #     port=right_arm_config['usb_port'],
    #     motors={
    #         "joint_1": [1, "xl330-m288"],
    #         "joint_2": [2, "xl330-m288"],
    #         "joint_3": [3, "xl330-m288"],
    #         "joint_4": [4, "xl330-m288"],
    #         "joint_5": [5, "xl330-m288"],
    #         "joint_6": [6, "xl330-m288"],
    #         "joint_7": [7, "xl330-m288"],
    #         "gripper": [8, "xl330-m288"],
    #     },
    # )

    left_arm_config = {}
    # left_arm_config['usb_port'] = "/dev/ttyUSB1"
    left_arm_config['ip'] = "169.254.128.18"
    left_arm_config['fps'] = 30
    # left_arm_config['calibration_dir'] = ".cache/calibration/adora_dual_left"
    # left_arm_config['start_pose'] = [-90.0, 90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
    # left_arm_config['joint_p_limit'] = [169.0, 102.0, 169.0, 52.0, 169.0, 117.0, 169.0]
    # left_arm_config['joint_n_limit'] = [-169.0, -102.0, -169.0, -167.0, -169.0, -87.0, -169.0]
    # left_leader_arm = DynamixelMotorsBusConfig(
    #     port=left_arm_config['usb_port'],
    #     motors={
    #         "joint_1": [1, "xl330-m288"],
    #         "joint_2": [2, "xl330-m288"],
    #         "joint_3": [3, "xl330-m288"],
    #         "joint_4": [4, "xl330-m288"],
    #         "joint_5": [5, "xl330-m288"],
    #         "joint_6": [6, "xl330-m288"],
    #         "joint_7": [7, "xl330-m288"],
    #         "gripper": [8, "xl330-m288"],
    #     },
    # )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "top": OpenCVCameraConfig(
                camera_index=20,
                fps=30,
                width=640,
                height=480,
            ),
            "left_wrist": OpenCVCameraConfig(
                camera_index=4,
                fps=30,
                width=640,
                height=480,
            ),
            "right_wrist": OpenCVCameraConfig(
                camera_index=22,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    mock: bool = False


@RobotConfig.register_subclass("pika_v1")
@dataclass
class PikaV1RobotConfig(RobotConfig):
    right_leader_arm = PikaMotorsBusConfig(
        port="right",
        motors={
            "pose_x":           [1, "pika-pose"],
            "pose_y":           [2, "pika-pose"],
            "pose_z":           [3, "pika-pose"],
            "rotation_quat_x":  [4, "pika-pose"],
            "rotation_quat_y":  [5, "pika-pose"],
            "rotation_quat_z":  [6, "pika-pose"],
            "rotation_quat_w":  [7, "pika-pose"],
            "gripper":          [8, "pika-gripper"],
        },
    )

    left_leader_arm = PiperMotorsBusConfig(
        port="left",
        motors={
            "pose_x":           [1, "pika-pose"],
            "pose_y":           [2, "pika-pose"],
            "pose_z":           [3, "pika-pose"],
            "rotation_quat_x":  [4, "pika-pose"],
            "rotation_quat_y":  [5, "pika-pose"],
            "rotation_quat_z":  [6, "pika-pose"],
            "rotation_quat_w":  [7, "pika-pose"],
            "gripper":          [8, "pika-gripper"],
        },
    )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "image_right": OpenCVCameraConfig(
                camera_index=1,
                fps=30,
                width=640,
                height=480,
            ),
            "image_left": OpenCVCameraConfig(
                camera_index=2,
                fps=30,
                width=640,
                height=480,
            ),
            "image_right_fisheye": OpenCVCameraConfig(
                camera_index=3,
                fps=30,
                width=640,
                height=480,
            ),
            "image_left_fisheye": OpenCVCameraConfig(
                camera_index=4,
                fps=30,
                width=640,
                height=480,
            ),
            "image_right_tac_r": OpenCVCameraConfig(
                camera_index=5,
                fps=30,
                width=640,
                height=480,
            ),
            "image_right_tac_l": OpenCVCameraConfig(
                camera_index=6,
                fps=30,
                width=640,
                height=480,
            ),
            "image_left_tac_r": OpenCVCameraConfig(
                camera_index=7,
                fps=30,
                width=640,
                height=480,
            ),
            "image_left_tac_l": OpenCVCameraConfig(
                camera_index=8,
                fps=30,
                width=640,
                height=480,
            ),
            "image_pika_pose": OpenCVCameraConfig(
                camera_index=9,
                fps=30,
                width=1280,
                height=960,
            ),
        }
    )
    
@RobotConfig.register_subclass("dexterous_hand_v1")  
@dataclass  
class DexterousHandRobotConfig(RobotConfig):
    left_full_skeleton = {  
        "point_1": [1, "finger-joint"],
        "point_2": [2, "finger-joint"],
        "point_3": [3, "finger-joint"],
        "point_4": [4, "finger-joint"],
        "point_5": [5, "finger-joint"],
        "point_6": [6, "finger-joint"],
        "point_7": [7, "finger-joint"],
        "point_8": [8, "finger-joint"],
        "point_9": [9, "finger-joint"],
        "point_10": [10, "finger-joint"],
        "point_11": [11, "finger-joint"],
        "point_12": [12, "finger-joint"],
        "point_13": [13, "finger-joint"],
        "point_14": [14, "finger-joint"],
        "point_15": [15, "finger-joint"],
        "point_16": [16, "finger-joint"],
        "point_17": [17, "finger-joint"],
        "point_18": [18, "finger-joint"],
        "point_19": [19, "finger-joint"],
        "point_20": [20, "finger-joint"],
        "point_21": [21, "finger-joint"],
        "point_22": [22, "finger-joint"],
        "point_23": [23, "finger-joint"],
        "point_24": [24, "finger-joint"],
        "point_25": [25, "finger-joint"],
        "point_26": [26, "finger-joint"],
        "point_27": [27, "finger-joint"],
        "point_28": [28, "finger-joint"],
        "point_29": [29, "finger-joint"],
        "point_30": [30, "finger-joint"],
        "point_31": [31, "finger-joint"],
        "point_32": [32, "finger-joint"],
        "point_33": [33, "finger-joint"],
        "point_34": [34, "finger-joint"],
        "point_35": [35, "finger-joint"],
        "point_36": [36, "finger-joint"],
        "point_37": [37, "finger-joint"],
        "point_38": [38, "finger-joint"],
        "point_39": [39, "finger-joint"],
        "point_40": [40, "finger-joint"],
        "point_41": [41, "finger-joint"],
        "point_42": [42, "finger-joint"],
        "point_43": [43, "finger-joint"],
        "point_44": [44, "finger-joint"],
        "point_45": [45, "finger-joint"],
        "point_46": [46, "finger-joint"],
        "point_47": [47, "finger-joint"],
        "point_48": [48, "finger-joint"],
        "point_49": [49, "finger-joint"],
        "point_50": [50, "finger-joint"],
        "point_51": [51, "finger-joint"],
        "point_52": [52, "finger-joint"],
        "point_53": [53, "finger-joint"],
        "point_54": [54, "finger-joint"],
        "point_55": [55, "finger-joint"],
        "point_56": [56, "finger-joint"],
        "point_57": [57, "finger-joint"],
        "point_58": [58, "finger-joint"],
        "point_59": [59, "finger-joint"],
        "point_60": [60, "finger-joint"],
        "point_61": [61, "finger-joint"],
        "point_62": [62, "finger-joint"],
        "point_63": [63, "finger-joint"],
        "point_64": [64, "finger-joint"],
        "point_65": [65, "finger-joint"],
        "point_66": [66, "finger-joint"],
        "point_67": [67, "finger-joint"],
        "point_68": [68, "finger-joint"],
        "point_69": [69, "finger-joint"],
        "point_70": [70, "finger-joint"],
        "point_71": [71, "finger-joint"],
        "point_72": [72, "finger-joint"],
        "point_73": [73, "finger-joint"],
        "point_74": [74, "finger-joint"],
        "point_75": [75, "finger-joint"],
    }

    right_full_skeleton = {  
        "point_1": [1, "finger-joint"],
        "point_2": [2, "finger-joint"],
        "point_3": [3, "finger-joint"],
        "point_4": [4, "finger-joint"],
        "point_5": [5, "finger-joint"],
        "point_6": [6, "finger-joint"],
        "point_7": [7, "finger-joint"],
        "point_8": [8, "finger-joint"],
        "point_9": [9, "finger-joint"],
        "point_10": [10, "finger-joint"],
        "point_11": [11, "finger-joint"],
        "point_12": [12, "finger-joint"],
        "point_13": [13, "finger-joint"],
        "point_14": [14, "finger-joint"],
        "point_15": [15, "finger-joint"],
        "point_16": [16, "finger-joint"],
        "point_17": [17, "finger-joint"],
        "point_18": [18, "finger-joint"],
        "point_19": [19, "finger-joint"],
        "point_20": [20, "finger-joint"],
        "point_21": [21, "finger-joint"],
        "point_22": [22, "finger-joint"],
        "point_23": [23, "finger-joint"],
        "point_24": [24, "finger-joint"],
        "point_25": [25, "finger-joint"],
        "point_26": [26, "finger-joint"],
        "point_27": [27, "finger-joint"],
        "point_28": [28, "finger-joint"],
        "point_29": [29, "finger-joint"],
        "point_30": [30, "finger-joint"],
        "point_31": [31, "finger-joint"],
        "point_32": [32, "finger-joint"],
        "point_33": [33, "finger-joint"],
        "point_34": [34, "finger-joint"],
        "point_35": [35, "finger-joint"],
        "point_36": [36, "finger-joint"],
        "point_37": [37, "finger-joint"],
        "point_38": [38, "finger-joint"],
        "point_39": [39, "finger-joint"],
        "point_40": [40, "finger-joint"],
        "point_41": [41, "finger-joint"],
        "point_42": [42, "finger-joint"],
        "point_43": [43, "finger-joint"],
        "point_44": [44, "finger-joint"],
        "point_45": [45, "finger-joint"],
        "point_46": [46, "finger-joint"],
        "point_47": [47, "finger-joint"],
        "point_48": [48, "finger-joint"],
        "point_49": [49, "finger-joint"],
        "point_50": [50, "finger-joint"],
        "point_51": [51, "finger-joint"],
        "point_52": [52, "finger-joint"],
        "point_53": [53, "finger-joint"],
        "point_54": [54, "finger-joint"],
        "point_55": [55, "finger-joint"],
        "point_56": [56, "finger-joint"],
        "point_57": [57, "finger-joint"],
        "point_58": [58, "finger-joint"],
        "point_59": [59, "finger-joint"],
        "point_60": [60, "finger-joint"],
        "point_61": [61, "finger-joint"],
        "point_62": [62, "finger-joint"],
        "point_63": [63, "finger-joint"],
        "point_64": [64, "finger-joint"],
        "point_65": [65, "finger-joint"],
        "point_66": [66, "finger-joint"],
        "point_67": [67, "finger-joint"],
        "point_68": [68, "finger-joint"],
        "point_69": [69, "finger-joint"],
        "point_70": [70, "finger-joint"],
        "point_71": [71, "finger-joint"],
        "point_72": [72, "finger-joint"],
        "point_73": [73, "finger-joint"],
        "point_74": [74, "finger-joint"],
        "point_75": [75, "finger-joint"],
    }

    # 手腕追踪器配置  
    left_wrist_tracker = {  
        "pose_x": [1, "wrist-pose"],  
        "pose_y": [2, "wrist-pose"],   
        "pose_z": [3, "wrist-pose"],  
        "rotation_quat_x": [4, "wrist-pose"],  
        "rotation_quat_y": [5, "wrist-pose"],  
        "rotation_quat_z": [6, "wrist-pose"],  
        "rotation_quat_w": [7, "wrist-pose"],  
    }  
      
    right_wrist_tracker = {  
        "pose_x": [1, "wrist-pose"],  
        "pose_y": [2, "wrist-pose"],  
        "pose_z": [3, "wrist-pose"],   
        "rotation_quat_x": [4, "wrist-pose"],  
        "rotation_quat_y": [5, "wrist-pose"],  
        "rotation_quat_z": [6, "wrist-pose"],  
        "rotation_quat_w": [7, "wrist-pose"],  
    }  
      
    # 手指传感器配置 - 每手6个关节  
    left_finger_sensors = {  
        "joint_1": [1, "finger-joint"],  
        "joint_2": [2, "finger-joint"],  
        "joint_3": [3, "finger-joint"],  
        "joint_4": [4, "finger-joint"],
        "joint_5": [5, "finger-joint"], 
        "joint_6": [6, "finger-joint"],
        "joint_7": [7, "finger-joint"],  
        "joint_8": [8, "finger-joint"],  
        "joint_9": [9, "finger-joint"],  
        "joint_10": [10, "finger-joint"],
        "joint_11": [11, "finger-joint"], 
        "joint_12": [12, "finger-joint"],
        "joint_13": [13, "finger-joint"],  
        "joint_14": [14, "finger-joint"],  
        "joint_15": [15, "finger-joint"],    
    }  
      
    right_finger_sensors = {  
        "joint_1": [1, "finger-joint"],  
        "joint_2": [2, "finger-joint"],  
        "joint_3": [3, "finger-joint"],  
        "joint_4": [4, "finger-joint"],
        "joint_5": [5, "finger-joint"], 
        "joint_6": [6, "finger-joint"],
        "joint_7": [7, "finger-joint"],  
        "joint_8": [8, "finger-joint"],  
        "joint_9": [9, "finger-joint"],  
        "joint_10": [10, "finger-joint"],
        "joint_11": [11, "finger-joint"], 
        "joint_12": [12, "finger-joint"],
        "joint_13": [13, "finger-joint"],  
        "joint_14": [14, "finger-joint"],  
        "joint_15": [15, "finger-joint"],  
 
    }  
      
    # 头部追踪器配置  
    head_tracker = {  
        "pose_x": [1, "head-pose"],  
        "pose_y": [2, "head-pose"],  
        "pose_z": [3, "head-pose"],  
        "rotation_quat_x": [4, "head-pose"],  
        "rotation_quat_y": [5, "head-pose"],  
        "rotation_quat_z": [6, "head-pose"],  
        "rotation_quat_w": [7, "head-pose"],  
    }  
      
    cameras: dict[str, CameraConfig] = field(  
        default_factory=lambda: {  
            "image": IntelRealSenseCameraConfig(  
                serial_number=337122071807,  
                fps=30,  
                width=640,  
                height=480,  
            ),  
        }  
    )
