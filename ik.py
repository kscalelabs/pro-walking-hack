"""Simple ik script for Stompy Pro."""
import time

import numpy as np
import pybullet as p
import pybullet_data

EEL_JOINT = "left_end_effector_joint"
EER_JOINT = "right_end_effector_joint"

# Initialize PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot_id = p.loadURDF("urdf/stompy_pro/robot.urdf", [0, 0, 0], useFixedBase=True)
p.setGravity(0, 0, -9.81)

joint_reference = {
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

goal_pos_left = np.array([-0.25, -0.25, 0])
goal_pos_right = np.array([-0.25, 0.35, 0])

joint_info = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name = info[1].decode("utf-8")
    joint_info[name] = {
        "index": i,
        "lower_limit": info[8],
        "upper_limit": info[9],
    }

p.resetBasePositionAndOrientation(robot_id, [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))

p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 1],
)

p.addUserDebugPoints([goal_pos_left], [[1, 0, 0]], pointSize=20)
p.addUserDebugPoints([goal_pos_right], [[0, 0, 1]], pointSize=20)

def inverse_kinematics(arm: str, goal_pos: np.ndarray) -> None:
    ee_id = joint_info[EEL_JOINT if arm == "left" else EER_JOINT]["index"]
    ee_chain = (
        eel_chain if arm == "left" else eer_chain
    )
    target_pos = goal_pos_left if arm == "left" else goal_pos_right
    print(target_pos)

    lower_limits = [joint_info[joint]["lower_limit"] for joint in ee_chain]
    upper_limits = [joint_info[joint]["upper_limit"] for joint in ee_chain]
    joint_ranges = [upper - lower for upper, lower in zip(upper_limits, lower_limits)]

    torso_pos, torso_orn = p.getBasePositionAndOrientation(robot_id)
    inv_torso_pos, inv_torso_orn = p.invertTransform(torso_pos, torso_orn)
    target_pos_local = p.multiplyTransforms(inv_torso_pos, inv_torso_orn, target_pos, [0, 0, 0, 1])[0]

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

    print(solution)
    print(len(solution))

    for i, val in enumerate(solution):
        joint_name = list(joint_reference.keys())[i]
        print(joint_name)
        if joint_name in ee_chain:
            p.resetJointState(robot_id, joint_info[joint_name]["index"], val)

    actual_pos, _ = p.getLinkState(robot_id, ee_id)[:2]
    error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
    print(error)

# Add sliders for controlling the end effector positions
left_x_slider = p.addUserDebugParameter("Left EE X", -1, 1, goal_pos_left[0])
left_y_slider = p.addUserDebugParameter("Left EE Y", -1, 1, goal_pos_left[1])
left_z_slider = p.addUserDebugParameter("Left EE Z", -1, 1, goal_pos_left[2])

right_x_slider = p.addUserDebugParameter("Right EE X", -1, 1, goal_pos_right[0])
right_y_slider = p.addUserDebugParameter("Right EE Y", -1, 1, goal_pos_right[1])
right_z_slider = p.addUserDebugParameter("Right EE Z", -1, 1, goal_pos_right[2])

while True:
    # Update goal positions based on slider values
    goal_pos_left = np.array([
        p.readUserDebugParameter(left_x_slider),
        p.readUserDebugParameter(left_y_slider),
        p.readUserDebugParameter(left_z_slider)
    ])

    goal_pos_right = np.array([
        p.readUserDebugParameter(right_x_slider),
        p.readUserDebugParameter(right_y_slider),
        p.readUserDebugParameter(right_z_slider)
    ])

    inverse_kinematics("left", goal_pos_left)
    inverse_kinematics("right", goal_pos_right)

    p.removeAllUserDebugItems()
    p.addUserDebugPoints([goal_pos_left], [[1, 0, 0]], pointSize=20)
    p.addUserDebugPoints([goal_pos_right], [[0, 0, 1]], pointSize=20)

    # Sleep to make the simulation more stable
    time.sleep(0.01)
