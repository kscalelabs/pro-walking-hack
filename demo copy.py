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
import yaml
import math
from typing import Dict, Any
from numpy.typing import NDArray


URDF_WEB = "https://raw.githubusercontent.com/kscalelabs/teleop/8d1818cb303b26e234307a99b1bff65dad93d140/urdf/stompy_pro/robot.urdf"

VUER_TRUNK_ROTATION = [-np.pi/2, 0, np.pi/2] # Vuer robot rotation
VUER_TRUNK_POSITION = np.array([0, 0.63*1.5, 0]) # Vuer robot position

PYBULLET_TRUNK_ROTATION = [0, 0, np.pi] # Pybullet robot rotation
PYBULLET_TRUNK_POSITION = np.array([0, 0, 1]) # Pybullet robot position

EEL_JOINT = "left_end_effector_joint" # Left arm end effector joint - used for IK
EER_JOINT = "right_end_effector_joint"

ROBOT_HEIGHT = 1.0
ROBOT_WIDTH = 0.5

MAX_JOINT_DELTA = 3 #0.5 # radians

# Constants for hand tracking
PB_TO_VUER_AXES = [2, 0, 1] # Maps vuer axes to pybullet axes
PB_TO_VUER_AXES_SIGN = np.array([1, 1, 1], dtype=np.int8) # Sign of vuer axes

# Hand tracking parameters
INDEX_FINGER_TIP_ID, THUMB_FINGER_TIP_ID, MIDDLE_FINGER_TIP_ID = 9, 4, 14 #8, 4, 14
PINCH_DIST_CLOSED, PINCH_DIST_OPENED = 0.1, 0.1     # 10 cm
MAX_PINCH_DIST = 0.12 #appox 12cm at max pinch

ROTATION_MAX = np.pi # 180 degrees max rotation range for wrist pitch

PB_TO_VUER_OFFSET = PYBULLET_TRUNK_POSITION - VUER_TRUNK_POSITION[PB_TO_VUER_AXES] * PB_TO_VUER_AXES_SIGN # Offset between pybullet and vuer trunk positions (used for mapping hand input to pybullet targets)

# TODO: Figure out how to map Pybullet joint positions to real joint positions
# Currently just zeroing at arms down position (like in default pybullet URDF)
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
    left_hand_pinch_dist: float = 0.0
    right_hand_pinch_dist: float = 0.0
    prev_solution_left: np.ndarray = field(default_factory=lambda: np.zeros(4))
    prev_solution_right: np.ndarray = field(default_factory=lambda: np.zeros(4))


def initialize_robot() -> tuple[RobstrideMotorsSupervisor, RobstrideMotorsSupervisor]:
    # From top down on the robot torso
    motor_config = {
        1: "03",
        2: "03",
        3: "01",
        4: "01",
        5: "01",
    }

    left_port = "/dev/ttyCH341USB0"
    right_port = "/dev/ttyCH341USB1"

    # Initialize motors
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

    # Add motors to zero
    for motor_id in range(1, 6):
        left_arm.add_motor_to_zero(motor_id)
        right_arm.add_motor_to_zero(motor_id)


    # Set PD constants and safety limits for 04 motors
    for motor_id in range(1, 3):
        left_arm.set_kp(motor_id, 30.0)
        right_arm.set_kp(motor_id, 30.0)

        left_arm.set_kd(motor_id, 1.0)
        right_arm.set_kd(motor_id, 1.0)

        left_arm.set_torque_limit(motor_id, 1.0)
        right_arm.set_torque_limit(motor_id, 1.0)

        left_arm.set_speed_limit(motor_id, 1.0) # rad/s
        right_arm.set_speed_limit(motor_id, 1.0)

    # Set PD constants and safety limits for 02 motors
    for motor_id in range(3, 6):
        left_arm.set_kp(motor_id, 10.0)
        right_arm.set_kp(motor_id, 10.0)

        left_arm.set_kd(motor_id, 0.1)
        right_arm.set_kd(motor_id, 0.1)

        left_arm.set_torque_limit(motor_id, 1.0)
        right_arm.set_torque_limit(motor_id, 1.0)

        left_arm.set_speed_limit(motor_id, 1.0) # rad/s
        right_arm.set_speed_limit(motor_id, 1.0)

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


def apply_ema(current: np.ndarray, previous: np.ndarray, alpha: float = 0.4) -> np.ndarray:
    """Apply Exponential Moving Average smoothing."""
    return alpha * current + (1 - alpha) * previous


class TeleopRobot:
    def __init__(
        self, real: bool = False, embodiment: str, use_firmware: bool = False, shared_dict: dict = {}
    ) -> None:
        self.app = Vuer()

        self.shared_state = VuerInput()
        self.real = real
        if self.real:
            left_arm, right_arm = initialize_robot()

        self.shared_data = shared_dict
        self.update_positions()
        self.update_shared_data()

    def update_shared_data(self) -> None:
        self.shared_data["positions"] = self.get_positions()
        self.shared_data["velocities"] = self.get_velocities()


    def update_positions(self) -> None:
        if self.robot:
            self.robot.update_motor_data()
            pos = self.robot.get_motor_positions()["right_arm"]
            self.positions = np.array(pos)

    def get_positions(self) -> dict[str, dict[str, NDArray]]:
        if self.robot:
            return {
                "expected": {
                    "left": np.array(
                        [
                            math.degrees(self.q[pos] - self.config["start_q"][pos])
                            for pos in self.eel_chain_arm + self.eel_chain_hand
                        ]
                    ),
                },
                "actual": {
                    "left": self.positions,
                },
            }
        else:
            return {
                "expected": {
                    "left": np.array([math.degrees(self.q[pos]) for pos in self.eel_chain_arm + self.eel_chain_hand]),
                },
                "actual": {
                    "left": np.random.rand(6),
                },
            }

    def get_velocities(self) -> Dict[str, NDArray]:
        return {
            "left": np.zeros(6),
        }
    
    def setup_pybullet(self, use_gui: bool, urdf_path: str) -> None:
        # Initialize PyBullet
        p.connect(p.GUI if use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        robot_id = p.loadURDF("urdf/stompy_pro/robot.urdf", [0, 0, 0], useFixedBase=True)
        p.setGravity(0, 0, -9.81)

        # Initial goal positions
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

    def run(
        self,
        use_gui: bool,
        max_fps: int,
        urdf_path: str,
    ) -> None:
        self.setup_pybullet(use_gui, urdf_path)

        @self.app.add_handler("HAND_MOVE")
        async def hand_move_wrapper(event: Any, _: Any) -> None:
            hand_move_handler(event, self.shared_state)

        @self.app.spawn(start=True)
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

            counter = 0

            while True:
                # Update goal positions based on hand tracking input
                goal_pos_left = shared_state.left_hand_position
                goal_pos_right = shared_state.right_hand_position

                p.addUserDebugPoints([goal_pos_left], [[1, 0, 0]], pointSize=20, lifeTime=0.25)
                p.addUserDebugPoints([goal_pos_right], [[0, 0, 1]], pointSize=20, lifeTime=0.25)

                if counter % 1 == 0:
                    raw_solution_left = inverse_kinematics("left", goal_pos_left)
                    raw_solution_right = inverse_kinematics("right", goal_pos_right)

                    # Apply EMA smoothing
                    shared_state.prev_solution_left = apply_ema(raw_solution_left, shared_state.prev_solution_left)
                    shared_state.prev_solution_right = apply_ema(raw_solution_right, shared_state.prev_solution_right)

                    solution_left = shared_state.prev_solution_left
                    solution_right = shared_state.prev_solution_right

                counter += 1

                # Send commands to real robot
                if self.real:
                    solution_left = np.add(solution_left, SIM_TO_REAL["left"])
                    solution_right = np.add(solution_right, SIM_TO_REAL["right"])

                    left_rotation = ((shared_state.left_hand_pinch_dist / MAX_PINCH_DIST) - 0.5) * ROTATION_MAX # remaps pinch dist (0 to 0.12) to (-pi/2 to pi/2)
                    right_rotation = ((shared_state.right_hand_pinch_dist / MAX_PINCH_DIST) - 0.5) * ROTATION_MAX

                    for i, val in enumerate(solution_left):
                        # Offset by 1 because the motors are 1-indexed
                        left_arm.set_position(i+1, val)

                    for i, val in enumerate(solution_right):
                        right_arm.set_position(i+1, val)

                    left_arm.set_position(5, left_rotation) # wrist pitch
                    right_arm.set_position(5, right_rotation)

                # Update Vuer with the current joint positions
                session.upsert @ Urdf(
                    src=URDF_WEB,
                    jointValues={name: p.getJointState(robot_id, joint_info[name]["index"])[0] for name in joint_info},
                    position=VUER_TRUNK_POSITION,
                    rotation=VUER_TRUNK_ROTATION,
                    key="robot",
                )

                # Sleep to make the simulation more stable
                await asyncio.sleep(1/frequency)


        self.app.run()


def run_teleop_app(
    config: Dict[str, Any],
    embodiment: str,
    use_gui: bool,
    max_fps: int,
    use_firmware: bool,
    shared_data: Dict[str, NDArray],
) -> None:
    pass
    # teleop = TeleopRobot(config, embodiment, use_firmware=use_firmware, shared_dict=shared_data)
    # teleop.run(use_gui, max_fps, config["embodiments"][embodiment]["urdf_local"])


def main() -> None:
    parser = argparse.ArgumentParser(description="PyBullet and Vuer integration for robot control")
    parser.add_argument("--firmware", action="store_true", help="Enable firmware control")
    parser.add_argument("--gui", action="store_true", help="Use PyBullet GUI mode")
    parser.add_argument("--fps", type=int, default=60, help="Maximum frames per second")
    parser.add_argument("--config", type=str, default="config.yaml", help="Path to configuration file")
    parser.add_argument(
        "--embodiment",
        type=str,
        default="stompy_mini",
        choices=["stompy_mini", "stompy"],
        help="Robot embodiment to use",
    )
    args = parser.parse_args()


    demo = TeleopRobot(config, args.embodiment, use_firmware=args.firmware)
    demo.run(args.gui, args.fps, config["embodiments"][args.embodiment]["urdf_local"])


if __name__ == "__main__":
    main()