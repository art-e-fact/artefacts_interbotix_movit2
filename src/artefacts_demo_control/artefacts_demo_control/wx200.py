from typing import List

MOVE_GROUP_ARM: str = "interbotix_arm"
MOVE_GROUP_GRIPPER: str = "interbotix_gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.037, 0.037]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.015, 0.015]

robot_prefix = "wx200/"

def joint_names() -> List[str]:
    return [
        "waist",
        "shoulder",
        "elbow",
        "wrist_angle",
        "wrist_rotate",
    ]

def base_link_name(prefix: str = robot_prefix) -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = robot_prefix) -> str:
    return prefix + "ee_gripper_link"


def gripper_joint_names(prefix: str = robot_prefix) -> List[str]:
    return [
        prefix + "left_finger_link",
        prefix + "right_finger_link",
    ]