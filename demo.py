"""Simple vr teleop script for Stompy Pro.

python demo.py --real

make sure to run `ngrok http 8012` to serve the vuer app over the network
"""
import argparse
import asyncio
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pybullet as p
import pybullet_data
from actuator import RobstrideMotorsSupervisor
from vuer import Vuer, VuerSession
from vuer.schemas import Hands, PointLight, Urdf


def inverse_kinematics(arm: str, goal_pos: np.ndarray) -> np.ndarray:
    """Performs inverse kinematics for the given arm."""
    ee_id = joint_info[EEL_JOINT if arm == "left" else EER_JOINT]["index"]
    ee_chain = (
        eel_chain if arm == "left" else eer_chain
    )

    lower_limits = [joint_info[joint]["lower_limit"] for joint in ee_chain]
    upper_limits = [joint_info[joint]["upper_limit"] for joint in ee_chain]
    joint_ranges = [upper - lower for upper, lower in zip(upper_limits, lower_limits)]

    torso_pos, torso_orn = p.getBasePositionAndOrientation(robot_id)
    inv_torso_pos, inv_torso_orn = p.invertTransform(torso_pos, torso_orn)
    target_pos_local = p.multiplyTransforms(inv_torso_pos, inv_torso_orn, goal_pos, [0, 0, 0, 1])[0]

    movable_joints = [
        j for j in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, j)[2] != p.JOINT_FIXED
    ]

    current_positions = [p.getJointState(robot_id, j)[0] for j in movable_joints]
    solution = p.calculateInverseKinematics(
        robot_id,
        ee_id,
        target_pos_local,
        currentPositions=current_positions,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=SIM_TO_REAL[arm],
    )

    # Safety check: Ensure the solution is within acceptable limits
    is_solution_safe = all(
        abs(current_positions[i] - solution[i]) <= MAX_JOINT_DELTA
        for i in range(len(solution))
    )

    if not is_solution_safe:
        print("IK solution exceeds safety limits, skipping update.")
        return np.array(current_positions)

    for i, val in enumerate(solution):
        joint_name = list(initial_positions.keys())[i]
        if joint_name in ee_chain:
            p.resetJointState(robot_id, joint_info[joint_name]["index"], val)
            # print(f"Setting {joint_name} to {val}")

    actual_pos, _ = p.getLinkState(robot_id, ee_id)[:2]
    error = np.linalg.norm(np.array(goal_pos) - np.array(actual_pos))

    # print(f"Arm: {arm}, Error: {error}")
    if arm == "left":
        return np.array(solution[:4]) * -1
    else:
        return np.array(solution[4:])

def hand_move_handler(event: Any, shared_state: VuerInput) -> None:
    """Handle hand movement events from Vuer."""
    # Right hand
    rthumb_pos = np.array(event.value["rightLandmarks"][THUMB_FINGER_TIP_ID])
    rindex_pos = np.array(event.value["rightLandmarks"][INDEX_FINGER_TIP_ID])
    rpinch_dist = np.linalg.norm(rindex_pos - rthumb_pos)
    shared_state.right_hand_pinch = rpinch_dist < PINCH_DIST_CLOSED
    shared_state.right_hand_pinch_dist = rpinch_dist
    if shared_state.right_hand_pinch:
        shared_state.right_hand_position = (
            np.multiply(rthumb_pos[PB_TO_VUER_AXES], PB_TO_VUER_AXES_SIGN) + PB_TO_VUER_OFFSET
        )

    # Left hand
    lthumb_pos = np.array(event.value["leftLandmarks"][THUMB_FINGER_TIP_ID])
    lindex_pos = np.array(event.value["leftLandmarks"][INDEX_FINGER_TIP_ID])
    lpinch_dist = np.linalg.norm(lindex_pos - lthumb_pos)
    shared_state.left_hand_pinch = lpinch_dist < PINCH_DIST_CLOSED
    shared_state.left_hand_pinch_dist = lpinch_dist
    if shared_state.left_hand_pinch:
        shared_state.left_hand_position = (
            np.multiply(lthumb_pos[PB_TO_VUER_AXES], PB_TO_VUER_AXES_SIGN) + PB_TO_VUER_OFFSET
        )
