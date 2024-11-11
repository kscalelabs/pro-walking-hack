"""Simple ik script for Stompy Pro."""
import argparse
from dataclasses import dataclass, field
import platform
import threading
import time
import asyncio
from typing import Any

import numpy as np
import pybullet as p
import pybullet_data
from actuator import RobstrideMotorsSupervisor
from vuer import Vuer, VuerSession
from vuer.schemas import Hands, PointLight, Urdf

URDF_WEB = "https://raw.githubusercontent.com/kscalelabs/teleop/d5960dfe1de14dffd3eb990ab8b4a1425bd62406/urdf/stompy_pro/robot.urdf"

VUER_TRUNK_ROTATION = [-np.pi/2, 0, np.pi/2]
VUER_TRUNK_POSITION = np.array([0, 0.63, 0])

PYBULLET_TRUNK_ROTATION = [0, 0, np.pi]
PYBULLET_TRUNK_POSITION = np.array([0, 0, 1])

EEL_JOINT = "left_end_effector_joint"
EER_JOINT = "right_end_effector_joint"

ROBOT_HEIGHT = 1.0
ROBOT_WIDTH = 0.5

MAX_JOINT_DELTA = 3 #0.5 # radians

# Constants for hand tracking
PB_TO_VUER_AXES = [2, 0, 1]
PB_TO_VUER_AXES_SIGN = np.array([1, 1, 1], dtype=np.int8)

# Hand tracking parameters
INDEX_FINGER_TIP_ID, THUMB_FINGER_TIP_ID, MIDDLE_FINGER_TIP_ID = 8, 4, 14
PINCH_DIST_CLOSED, PINCH_DIST_OPENED = 0.1, 0.1  # 10 cm
EE_S_MIN, EE_S_MAX = 0.0, 0.05

PB_TO_VUER_OFFSET = PYBULLET_TRUNK_POSITION - VUER_TRUNK_POSITION[PB_TO_VUER_AXES] * PB_TO_VUER_AXES_SIGN
# TODO: Figure out how to map Pybullet joint positions to real joint positions
SIM_TO_REAL = {
    "left": np.array([0,
                      0,
                      0,
                      0]),
    "right": np.array([0,
                       0,
                       0,
                       0]),
}

initial_positions = {
    "L_shoulder_y": 0,
    "L_shoulder_x": 0,
    "L_shoulder_z": 0,
    "L_elbow_x": 0,
    "R_shoulder_y": 0,
    "R_shoulder_x": 0,
    "R_shoulder_z": 0,
    "R_elbow_x": 0,
}

eer_chain = [
    "R_shoulder_y",
    "R_shoulder_x",
    "R_shoulder_z",
    "R_elbow_x",
]

eel_chain = [
    "L_shoulder_y",
    "L_shoulder_x",
    "L_shoulder_z",
    "L_elbow_x",
]

@dataclass
class VuerInput:
    left_hand_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    right_hand_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    left_hand_pinch: bool = False
    right_hand_pinch: bool = False

def initialize_robot() -> tuple[RobstrideMotorsSupervisor, RobstrideMotorsSupervisor]:
    motor_config = {
        1: "03",
        2: "03",
        3: "01",
        4: "01",
        5: "01",
    }

    left_port = "/dev/ttyCH341USB1"
    right_port = "/dev/ttyCH341USB0"

    left_arm = RobstrideMotorsSupervisor(
        port_name=left_port,
        motor_infos=motor_config,
        target_update_rate=10000.0,
    )
    right_arm = RobstrideMotorsSupervisor(
        port_name=right_port,
        motor_infos=motor_config,
        target_update_rate=10000.0,
    )

    for motor_id in range(1, 6):
        left_arm.add_motor_to_zero(motor_id)
        right_arm.add_motor_to_zero(motor_id)

    for motor_id in range(1, 3):
        left_arm.set_kp(motor_id, 60.0)
        right_arm.set_kp(motor_id, 60.0)

        left_arm.set_kd(motor_id, 5.0)
        right_arm.set_kd(motor_id, 5.0)
    
    for motor_id in range(3, 6):
        left_arm.set_kp(motor_id, 10.0)
        right_arm.set_kp(motor_id, 10.0)

        left_arm.set_kd(motor_id, 0.1)
        right_arm.set_kd(motor_id, 0.1)

    return left_arm, right_arm

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
    rpinch_dist = np.linalg.norm(np.array(event.value["rightLandmarks"][INDEX_FINGER_TIP_ID]) - rthumb_pos)
    shared_state.right_hand_pinch = rpinch_dist < PINCH_DIST_CLOSED
    if shared_state.right_hand_pinch:
        shared_state.right_hand_position = (
            np.multiply(rthumb_pos[PB_TO_VUER_AXES], PB_TO_VUER_AXES_SIGN) + PB_TO_VUER_OFFSET
        )

    # Left hand
    lthumb_pos = np.array(event.value["leftLandmarks"][THUMB_FINGER_TIP_ID])
    lpinch_dist = np.linalg.norm(np.array(event.value["leftLandmarks"][INDEX_FINGER_TIP_ID]) - lthumb_pos)
    shared_state.left_hand_pinch = lpinch_dist < PINCH_DIST_CLOSED
    if shared_state.left_hand_pinch:
        shared_state.left_hand_position = (
            np.multiply(lthumb_pos[PB_TO_VUER_AXES], PB_TO_VUER_AXES_SIGN) + PB_TO_VUER_OFFSET
        )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--real", action="store_true")
    args = parser.parse_args()

    if args.real:
        left_arm, right_arm = initialize_robot()

    # Initialize PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF("urdf/stompy_pro/robot.urdf", [0, 0, 0], useFixedBase=True)
    p.setGravity(0, 0, -9.81)

    goal_pos_left = np.array([0, 0, 0]) + np.array([0, ROBOT_WIDTH / 2, ROBOT_HEIGHT])
    goal_pos_right = np.array([0, 0, 0]) + np.array([0, -ROBOT_WIDTH / 2, ROBOT_HEIGHT])

    # Load joint info
    joint_info = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode("utf-8")
        joint_info[name] = {
            "index": i,
            "lower_limit": info[8],
            "upper_limit": info[9],
        }

        # Set initial positions
        if name in initial_positions:
            p.resetJointState(robot_id, i, initial_positions[name])

    p.resetBasePositionAndOrientation(robot_id, PYBULLET_TRUNK_POSITION, p.getQuaternionFromEuler(PYBULLET_TRUNK_ROTATION))

    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=50,
        cameraPitch=-35,
        cameraTargetPosition=[0, 0, 1],
    )

    app = Vuer()
    shared_state = VuerInput()

    @app.add_handler("HAND_MOVE")
    async def hand_move_wrapper(event: Any, _: Any) -> None:
        await asyncio.sleep(2)
        hand_move_handler(event, shared_state)

    @app.spawn(start=True)
    async def app_main(session: VuerSession) -> None:
        session.upsert @ PointLight(intensity=10.0, position=[0, 2, 2])
        session.upsert @ Hands(fps=30, stream=True, key="hands")
        await asyncio.sleep(0.1)
        session.upsert @ Urdf(
            src=URDF_WEB,
            jointValues=initial_positions,
            position=VUER_TRUNK_POSITION,
            rotation=VUER_TRUNK_ROTATION,
            key="robot",
        )
        
        while True:
            # Update goal positions based on hand tracking input
            goal_pos_left = shared_state.left_hand_position
            goal_pos_right = shared_state.right_hand_position

            p.addUserDebugPoints([goal_pos_left], [[1, 0, 0]], pointSize=20, lifeTime=0.25)
            p.addUserDebugPoints([goal_pos_right], [[0, 0, 1]], pointSize=20, lifeTime=0.25)

            solution_left = inverse_kinematics("left", goal_pos_left)
            solution_right = inverse_kinematics("right", goal_pos_right)

            if args.real:
                solution_left = np.add(solution_left, SIM_TO_REAL["left"])
                solution_right = np.add(solution_right, SIM_TO_REAL["right"])

                for i, val in enumerate(solution_left):
                    left_arm.set_position(i+1, val)

                for i, val in enumerate(solution_right):
                    right_arm.set_position(i+1, val)

            # Update Vuer with the current joint positions
            session.upsert @ Urdf(
                src=URDF_WEB,
                jointValues={name: p.getJointState(robot_id, joint_info[name]["index"])[0] for name in joint_info},
                position=VUER_TRUNK_POSITION,
                rotation=VUER_TRUNK_ROTATION,
                key="robot",
            )

            # Sleep to make the simulation more stable
            await asyncio.sleep(0.025)

    app.run()
