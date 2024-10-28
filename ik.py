"""Simple ik script for Stompy Pro."""
import argparse
import platform
import time

import numpy as np
import pybullet as p
import pybullet_data
# from actuator import RobstrideMotorsSupervisor
from inputs import get_gamepad

EEL_JOINT = "left_end_effector_joint"
EER_JOINT = "right_end_effector_joint"

MAX_JOINT_DELTA = 0.5 # radians

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

# def initialize_robot() -> tuple[RobstrideMotorsSupervisor, RobstrideMotorsSupervisor]:
#     motor_config = {
#         1: "04",
#         2: "04",
#         3: "04",
#         4: "04",
#     }

#     left_port = "/dev/ttyCH341USB0"
#     right_port = "/dev/ttyCH341USB1"

#     left_arm = RobstrideMotorsSupervisor(
#         port_name=left_port,
#         motor_infos=motor_config,
#         target_update_rate=10000.0,
#     )
#     right_arm = RobstrideMotorsSupervisor(
#         port_name=right_port,
#         motor_infos=motor_config,
#         target_update_rate=10000.0,
#     )

#     return left_arm, right_arm

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

    actual_pos, _ = p.getLinkState(robot_id, ee_id)[:2]
    error = np.linalg.norm(np.array(goal_pos) - np.array(actual_pos))

    print(f"Arm: {arm}, Goal : {error}")

    return np.array(solution)

def read_controller_input() -> tuple[float, float, float, bool]:
    """Reads input from the Logitech controller and returns changes in position and button state."""
    x_change, y_change, z_change = 0, 0, 0
    toggle_button_pressed = False
    events = get_gamepad()
    for event in events:
        if event.ev_type == 'Absolute':
            if event.code == 'ABS_X':  # Left stick horizontal
                x_change = event.state / 32768.0  # Normalize the value
            elif event.code == 'ABS_Y':  # Left stick vertical
                y_change = event.state / 32768.0
            elif event.code == 'ABS_RZ':  # Right stick vertical
                z_change = event.state / 32768.0
        elif event.ev_type == 'Key' and event.code == 'BTN_SOUTH' and event.state == 1:
            toggle_button_pressed = True
    return x_change, y_change, z_change, toggle_button_pressed


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

    goal_pos_left = np.array([-0.25, -0.25, 0])
    goal_pos_right = np.array([-0.25, 0.35, 0])

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

    p.resetBasePositionAndOrientation(robot_id, [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))

    p.resetDebugVisualizerCamera(
        cameraDistance=2.0,
        cameraYaw=50,
        cameraPitch=-35,
        cameraTargetPosition=[0, 0, 1],
    )

    if platform.system() == "Linux":
        # Initialize which arm to control
        control_left_arm = True

        while True:
            # Read controller input
            x_change, y_change, z_change, toggle_button_pressed = read_controller_input()

            # Toggle control between left and right arm
            if toggle_button_pressed:
                control_left_arm = not control_left_arm
                time.sleep(0.2)  # Debounce delay to prevent rapid toggling

            # Update goal positions based on controller input
            if control_left_arm:
                goal_pos_left += np.array([x_change, y_change, z_change]) * 0.01
            else:
                goal_pos_right += np.array([x_change, y_change, z_change]) * 0.01

            p.addUserDebugPoints([goal_pos_left], [[1, 0, 0]], pointSize=20)
            p.addUserDebugPoints([goal_pos_right], [[0, 0, 1]], pointSize=20)

            solution_left = inverse_kinematics("left", goal_pos_left)
            solution_right = inverse_kinematics("right", goal_pos_right)

            if args.real:
                solution_left = np.add(solution_left, SIM_TO_REAL["left"])
                solution_right = np.add(solution_right, SIM_TO_REAL["right"])

                for i, val in enumerate(solution_left):
                    left_arm.set_position(i, val)

                for i, val in enumerate(solution_right):
                    right_arm.set_position(i, val)

            # Sleep to make the simulation more stable
            time.sleep(0.01)
    elif platform.system() == "Darwin":  # macOS
        # Create sliders for macOS
        left_x_slider = p.addUserDebugParameter("Left X", -1, 1, 0)
        left_y_slider = p.addUserDebugParameter("Left Y", -1, 1, 0)
        left_z_slider = p.addUserDebugParameter("Left Z", 0, 2, 1)
        right_x_slider = p.addUserDebugParameter("Right X", -1, 1, 0)
        right_y_slider = p.addUserDebugParameter("Right Y", -1, 1, 0)
        right_z_slider = p.addUserDebugParameter("Right Z", 0, 2, 1)

        while True:
            # Read slider values
            left_x = p.readUserDebugParameter(left_x_slider)
            left_y = p.readUserDebugParameter(left_y_slider)
            left_z = p.readUserDebugParameter(left_z_slider)
            right_x = p.readUserDebugParameter(right_x_slider)
            right_y = p.readUserDebugParameter(right_y_slider)
            right_z = p.readUserDebugParameter(right_z_slider)

            # Update goal positions based on slider input
            goal_pos_left = np.array([left_x, left_y, left_z])
            goal_pos_right = np.array([right_x, right_y, right_z])

            p.addUserDebugPoints([goal_pos_left], [[1, 0, 0]], pointSize=20)
            p.addUserDebugPoints([goal_pos_right], [[0, 0, 1]], pointSize=20)

            solution_left = inverse_kinematics("left", goal_pos_left)
            solution_right = inverse_kinematics("right", goal_pos_right)

            if args.real:
                solution_left = np.add(solution_left, SIM_TO_REAL["left"])
                solution_right = np.add(solution_right, SIM_TO_REAL["right"])

                for i, val in enumerate(solution_left):
                    left_arm.set_position(i, val)

                for i, val in enumerate(solution_right):
                    right_arm.set_position(i, val)

            # Sleep to make the simulation more stable
            time.sleep(0.01)
