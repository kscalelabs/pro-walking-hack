"""COPIED FROM WALKING HACK - ADAPT TO JOYCON AND KOS"""
import time
from dataclasses import dataclass

import numpy as np
from actuator import RobstrideMotorsSupervisor
import subprocess
from openlch import HAL
import loggingcd 

LEFT_LEG_IDS = [5, 4, 3, 2, 1]
RIGHT_LEG_IDS = [10, 9, 8, 7, 6]

ROBOT_HEIGHT = 1.0
ROBOT_WIDTH = 0.5

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
    
def get_servo_angles(hal) -> tuple[list[float], list[float]]:
    servo_states = hal.servo.get_positions()
    print(servo_states)
    servo_states = [i for i in servo_states if i[0] in available_servos]
    return ([servo_states[i] for i in LEFT_LEG_IDS], [servo_states[i] for i in RIGHT_LEG_IDS])

def initialize_robot() -> tuple[RobstrideMotorsSupervisor, RobstrideMotorsSupervisor]:

    motor_config = {
        1: "04",
        2: "03",
        3: "03",
        4: "04",
        5: "02",
    }

    left_port = "/dev/ttyCH341USB1"
    right_port = "/dev/ttyCH341USB0"

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
        left_leg.set_kp(motor_id, 60.0)
        right_leg.set_kp(motor_id, 60.0)

        left_leg.set_kd(motor_id, 5.0)
        right_leg.set_kd(motor_id, 5.0)
    
    for motor_id in range(3, 6):
        left_leg.set_kp(motor_id, 10.0)
        right_leg.set_kp(motor_id, 10.0)

        left_leg.set_kd(motor_id, 0.1)
        right_leg.set_kd(motor_id, 0.1)

    return left_leg, right_leg


if __name__ == "__main__":

    # Configure the logger
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)

    left_leg, right_leg = initialize_robot()
    hal = HAL()
    check_connection(logger)
    available_servos = hal.servo.scan()
    logger.info(f"Available servos: {available_servos}")

    while True:
        left_angles, right_angles = get_servo_angles(hal)
        print(f"left_angles: {left_angles}" )
        print(f"right_angles: {right_angles}")

        # for i in range(1, 5):
        #     left_leg.set_position(i, np.radians(left_angles[i-1]))
        #     right_leg.set_position(i, np.radians(right_angles[i-1]))

    # Sleep to make the simulation more stable
    time.sleep(0.025)
    