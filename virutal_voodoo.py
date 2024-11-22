"""COPIED FROM WALKING HACK - ADAPT TO JOYCON AND KOS"""
import logging
import subprocess
import time
from dataclasses import dataclass

import numpy as np
import pybullet as p
import pybullet_data
from actuator import RobstrideMotorsSupervisor
from openlch import HAL
import argparse

ROBOT_HEIGHT = 1.0
ROBOT_WIDTH = 0.5

LEFT_LEG_IDS = [5, 4, 3, 2, 1]
RIGHT_LEG_IDS = [10, 9, 8, 7, 6]

initial_positions = {
    "L_hip_y" : 0,
    "L_hip_x" : 0,
    "L_hip_z" : 0,
    "L_knee" : 0,
    "L_ankle_y" : 0,
    "R_hip_y" : 0,
    "R_hip_x" : 0,
    "R_hip_z" : 0,
    "R_knee" : 0,
    "R_ankle_y" : 0,
}

base_offsets = {
    "hip_y": 0,
    "hip_x": 45,
    "hip_z": 0,
    "knee": 0,
    "ankle_y": 0,
}

NEGATE_RIGHT = -1

angle_offsets = {
    "L_hip_y": base_offsets["hip_y"],
    "L_hip_x": base_offsets["hip_x"],
    "L_hip_z": base_offsets["hip_z"],
    "L_knee": base_offsets["knee"],
    "L_ankle_y": base_offsets["ankle_y"],
    "R_hip_y": NEGATE_RIGHT * base_offsets["hip_y"],
    "R_hip_x": NEGATE_RIGHT * base_offsets["hip_x"],
    "R_hip_z": NEGATE_RIGHT * base_offsets["hip_z"],
    "R_knee": NEGATE_RIGHT * base_offsets["knee"],
    "R_ankle_y": NEGATE_RIGHT * base_offsets["ankle_y"],
}

eer_chain = [
    "R_hip_y",
    "R_hip_x",
    "R_hip_z",
    "R_knee",
    "R_ankle_y",
]

eel_chain = [
    "L_hip_y",
    "L_hip_x",
    "L_hip_z",
    "L_knee",
    "L_ankle_y",
]

def check_connection(logger):
    """Checks the connection to the robot."""
    logger.info("Checking connection to robot...")
    try:
        subprocess.run(
            ["ping", "-c", "1", "192.168.42.1"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True
        )
        logger.info("Successfully pinged robot.")
    except subprocess.CalledProcessError:
        logger.error("Could not ping robot at 192.168.42.1")
        raise ConnectionError("Robot connection failed.")

def get_servo_angles(hal: HAL) -> tuple[list[float], list[float]]:
    servo_states = hal.servo.get_positions()
    # print(servo_states)
    # servo_states = [i for i in servo_states if i[0] in LEFT_LEG_IDS + RIGHT_LEG_IDS]
    # print(f"servo_states: {servo_states}")
    left_angles = []
    for i in LEFT_LEG_IDS:
        left_angles.append(next(state[1] for state in servo_states if state[0] == i))
    right_angles = []
    for i in RIGHT_LEG_IDS:
        right_angles.append(next(state[1] for state in servo_states if state[0] == i))
    return (left_angles, right_angles)

def initialize_robot() -> tuple[RobstrideMotorsSupervisor, RobstrideMotorsSupervisor]:
    motor_config = {
        1: "04",
        2: "03",
        3: "03",
        4: "04",
        5: "01",
    }

    left_port = "/dev/ttyCH341USB0"
    right_port = "/dev/ttyCH341USB1"

    left_leg = RobstrideMotorsSupervisor(
        port_name=left_port,
        motor_infos=motor_config,
        target_update_rate=10000.0,
    )
    right_leg = RobstrideMotorsSupervisor(
        port_name=right_port,
        motor_infos=motor_config,
        target_update_rate=10000.0,
    )

    for motor_id in range(1, 6):
        left_leg.add_motor_to_zero(motor_id)
        right_leg.add_motor_to_zero(motor_id)

    for motor_id in range(1, 3):
        left_leg.set_kp(motor_id, 0.1)
        right_leg.set_kp(motor_id, 60.0)

        left_leg.set_kd(motor_id, 0.0)
        right_leg.set_kd(motor_id, 0.0)

    for motor_id in range(3, 6):
        left_leg.set_kp(motor_id, 0.1)
        right_leg.set_kp(motor_id, 0.1)

        left_leg.set_kd(motor_id, 0.0)
        right_leg.set_kd(motor_id, 0.0)

    return left_leg, right_leg


if __name__ == "__main__":
    # Add argument parser
    parser = argparse.ArgumentParser(description='Virtual voodoo simulation with optional real hardware control')
    parser.add_argument('--real', action='store_true', help='Enable real hardware control')
    args = parser.parse_args()

    # Configure the logger
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)

    # Initialize real hardware only if --real flag is present
    left_leg = right_leg = None
    if args.real:
        left_leg, right_leg = initialize_robot()
    hal = HAL()
    check_connection(logger)
    available_servos = hal.servo.scan()
    logger.info(f"Available servos: {available_servos}")

    # Initialize PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF("urdf/kbot/robot.urdf", [0, 0, 0], useFixedBase=True)
    p.setGravity(0, 0, -9.81)

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

    while True:
        left_angles, right_angles = get_servo_angles(hal)

        for chain, leg, angles in [(eel_chain, left_leg, left_angles), (eer_chain, right_leg, right_angles)]:

            for i in range(len(chain)):
                joint_name = chain[i]
                angle = np.radians(angles[i])

                offset = np.radians(angle_offsets.get(joint_name))

                angle = offset + angle
                print(joint_name)
                angle = -1 * angle
                if joint_name in ["L_hip_x", "R_hip_y", "L_knee", "L_ankle_y", "R_ankle_y", "R_hip_z", "L_hip_z"]:
                    # print(f"negating {joint_name}")
                    angle = -1 * angle
                p.resetJointState(robot_id, joint_info[joint_name]["index"], angle)
                # Only set position if in real mode
                if args.real and leg is not None:
                    leg.set_position(i + 1, angle)

    # Sleep to make the simulation more stable
    time.sleep(0.025)
