from typing import List

MOVE_GROUP_ARM: str = "right_arm"
MOVE_GROUP_GRIPPER: str = "right_hand"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.020833, -0.020833]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "right_") -> List[str]:
    return [
        prefix + "s0",
        prefix + "s1",
        prefix + "e0",
        prefix + "e1",
        prefix + "w0",
        prefix + "w1",
        prefix + "w2",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "world"


def end_effector_name(prefix: str = "right_") -> str:
    return prefix + "gripper"


def gripper_joint_names(prefix: str = "r_") -> List[str]:
    return [
        prefix + "gripper_l_finger_joint",
        prefix + "gripper_r_finger_joint",
    ]
