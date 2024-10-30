"""Use onnxruntime to run a model."""

import threading
import time
from dataclasses import dataclass, field

import can
import numpy as np
import onnxruntime as ort
from actuator import RobstrideMotorFeedback, RobstrideMotorsSupervisor
from imu import HexmoveImuReader

# PD_CONSTANTS = {
#     1: {
#         "name": "hip_y",
#         "p": 300,
#         "d": 9,
#     },
#     2: {
#         "name": "hip_x",
#         "p": 240,
#         "d": 4,
#     },
#     3: {
#         "name": "hip_z",
#         "p": 200,
#         "d": 10,
#     },
#     4: {
#         "name": "knee",
#         "p": 270,
#         "d": 9,
#     },
#     5: {
#         "name": "ankle_y",
#         "p": 300,
#         "d": 9,
#     },
# }

PD_CONSTANTS = {
    1: {
        "name": "hip_y",
        "p": 40,
        "d": 1,
    },
    2: {
        "name": "hip_x",
        "p": 40,
        "d": 1,
    },
    3: {
        "name": "hip_z",
        "p": 40,
        "d": 1,
    },
    4: {
        "name": "knee",
        "p": 40,
        "d": 1,
    },
    5: {
        "name": "ankle_y",
        "p": 40,
        "d": 1,
    },
}

@dataclass
class JointData:
    position: float = 0.0
    velocity: float = 0.0

@dataclass
class LegData:
    hip_y: JointData = field(default_factory=JointData)
    hip_x: JointData = field(default_factory=JointData)
    hip_z: JointData = field(default_factory=JointData)
    knee: JointData = field(default_factory=JointData)
    ankle_y: JointData = field(default_factory=JointData)

    def set_feedback(self, feedback: list[RobstrideMotorFeedback]) -> None:
        self.hip_y.position = feedback[0].position
        self.hip_x.position = feedback[1].position
        self.hip_z.position = feedback[2].position
        self.knee.position = feedback[3].position
        self.ankle_y.position = feedback[4].position

        self.hip_y.velocity = feedback[0].velocity
        self.hip_x.velocity = feedback[1].velocity
        self.hip_z.velocity = feedback[2].velocity
        self.knee.velocity = feedback[3].velocity
        self.ankle_y.velocity = feedback[4].velocity

@dataclass
class ArmData:
    shoulder_y: JointData = field(default_factory=JointData)
    shoulder_x: JointData = field(default_factory=JointData)
    shoulder_z: JointData = field(default_factory=JointData)
    elbow: JointData = field(default_factory=JointData)
    wrist: JointData = field(default_factory=JointData)

    def set_feedback(self, feedback: list[RobstrideMotorFeedback]) -> None:
        try:
            self.shoulder_y.position = feedback[0].position
            self.shoulder_x.position = feedback[1].position
            self.shoulder_z.position = feedback[2].position
            self.elbow.position = feedback[3].position
            self.wrist.position = feedback[4].position
        except IndexError:
            print(f"Feedback length is not 5: {len(feedback)}")

@dataclass
class RobotData:
    left_leg: LegData = field(default_factory=LegData)
    right_leg: LegData = field(default_factory=LegData)
    left_arm: ArmData = field(default_factory=ArmData)
    right_arm: ArmData = field(default_factory=ArmData)


def run_onnx_model() -> None:
    session = ort.InferenceSession("standing.onnx")

    # Initialize the motors.
    leg_motor_infos = {
        1: "04",
        2: "03",
        3: "03",
        4: "04",
        5: "02",
    }

    # Initialize CAN-based IMU
    """
       sudo ip link set can0 up type can bitrate 500000
       ip link 
    """
    imu_reader = HexmoveImuReader("can0", 1, 1)

    # Converting from real to sim
    euler_signs = np.array([1, 1, 1])

    time.sleep(1)

    imu_data = imu_reader.get_data()

    # IMU offset
    angular_offset = np.array([imu_data.x_angle, imu_data.y_angle, imu_data.z_angle])

    left_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB0", motor_infos=leg_motor_infos,
                                        target_update_rate=10000.0)

    right_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB1", motor_infos=leg_motor_infos,
                                         target_update_rate=10000.0)

    robot_data = RobotData()

    for i in range(5):
        left_leg.add_motor_to_zero(i + 1)
        right_leg.add_motor_to_zero(i + 1)
        time.sleep(0.01)

    for motor_id, motor_info in PD_CONSTANTS.items():
        assert isinstance(motor_info["p"], float)
        assert isinstance(motor_info["d"], float)

        left_leg.set_kp(motor_id, motor_info["p"])
        right_leg.set_kp(motor_id, motor_info["p"])

        time.sleep(0.01)

        left_leg.set_kd(motor_id, motor_info["d"])
        right_leg.set_kd(motor_id, motor_info["d"])

    input_data = {
        "x_vel.1": np.zeros(1, dtype=np.float32),
        "y_vel.1": np.zeros(1, dtype=np.float32),
        "rot.1": np.zeros(1, dtype=np.float32),
        "t.1": np.zeros(1, dtype=np.float32),
        "dof_pos.1": np.zeros(10, dtype=np.float32),
        "dof_vel.1": np.zeros(10, dtype=np.float32),
        "prev_actions.1": np.zeros(10, dtype=np.float32),
        "imu_ang_vel.1": np.zeros(3, dtype=np.float32),
        "imu_euler_xyz.1": np.zeros(3, dtype=np.float32),
        "buffer.1": np.zeros(574, dtype=np.float32),
    }

    command = {"x_vel": 0.0,
               "y_vel": 0.0,
               "rot": 0.0,}

    input_data["x_vel.1"][0] = np.float32(command["x_vel"])
    input_data["y_vel.1"][0] = np.float32(command["y_vel"])
    input_data["rot.1"][0] = np.float32(command["rot"])

    def get_angular_velocity() -> np.ndarray:
        return np.array([imu_data.x_velocity, imu_data.y_velocity, imu_data.z_velocity])

    def get_euler_angles() -> np.ndarray:
        return np.deg2rad([imu_data.x_angle, imu_data.y_angle, imu_data.z_angle] - angular_offset) * euler_signs

    def update_motor_data() -> None:
        left_leg_feedback = left_leg.get_latest_feedback()
        right_leg_feedback = right_leg.get_latest_feedback()

        if len(left_leg_feedback) == 5:
            robot_data.left_leg.set_feedback([left_leg_feedback[i] for i in [1, 2, 3, 4, 5]])
        else:
            print(f"Left leg feedback length is not 5: {len(left_leg_feedback)}")
            print(f"Left leg feedback: {left_leg_feedback}")

        if len(right_leg_feedback) == 5:
            robot_data.right_leg.set_feedback([right_leg_feedback[i] for i in [1, 2, 3, 4, 5]])
        else:
            print(f"Right leg feedback length is not 5: {len(right_leg_feedback)}")

    def get_joint_angles() -> np.ndarray:
        return np.array([
            robot_data.left_leg.hip_y.position,
            robot_data.left_leg.hip_x.position,
            robot_data.left_leg.hip_z.position,
            robot_data.left_leg.knee.position,
            robot_data.left_leg.ankle_y.position,
            robot_data.right_leg.hip_y.position,
            robot_data.right_leg.hip_x.position,
            robot_data.right_leg.hip_z.position,
            robot_data.right_leg.knee.position,
            robot_data.right_leg.ankle_y.position,
        ])

    def get_joint_velocities() -> np.ndarray:
        return np.array([
            robot_data.left_leg.hip_y.velocity,
            robot_data.left_leg.hip_x.velocity,
            robot_data.left_leg.hip_z.velocity,
            robot_data.left_leg.knee.velocity,
            robot_data.left_leg.ankle_y.velocity,
            robot_data.right_leg.hip_y.velocity,
            robot_data.right_leg.hip_x.velocity,
            robot_data.right_leg.hip_z.velocity,
            robot_data.right_leg.knee.velocity,
            robot_data.right_leg.ankle_y.velocity,
        ])

    def send_positions(positions: np.ndarray) -> None:
        for motor_id, angle in enumerate(positions[:5]):
            left_leg.set_position(motor_id + 1, angle)
            time.sleep(0.01)

        for motor_id, angle in enumerate(positions[5:]):
            right_leg.set_position(motor_id + 1, angle)
            time.sleep(0.01)

    start_time = time.time()
    target_cycle_time = 1 / 50  # 50 Hz cycle rate

    actions = np.zeros(10, dtype=np.double)
    buffer = np.zeros(574, dtype=np.double)
    while True:
        cycle_start_time = time.time()
        elapsed_time = cycle_start_time - start_time

        update_motor_data()
        imu_data = imu_reader.get_data()

        input_data["t.1"][0] = np.float32(elapsed_time)
        input_data["dof_pos.1"] = get_joint_angles().astype(np.float32)
        input_data["dof_vel.1"] = get_joint_velocities().astype(np.float32)
        input_data["imu_ang_vel.1"] = get_angular_velocity().astype(np.float32)
        input_data["imu_euler_xyz.1"] = get_euler_angles().astype(np.float32)

        input_data["prev_actions.1"] = actions
        input_data["buffer.1"] = buffer

        positions, actions, buffer = session.run(None, input_data)

        if np.any(positions > 0.1) or np.any(positions < -0.1):
            print(f"Positions out of bounds: {positions}")
            positions = np.clip(positions, -0.1, 0.1) 
            print(f"Dof_pos: {input_data['dof_pos.1']}")

        send_positions(positions)

        cycle_end_time = time.time()
        cycle_duration = cycle_end_time - cycle_start_time
        sleep_time = max(0, target_cycle_time - cycle_duration)
        time.sleep(sleep_time)
        # print(f"Slept for {sleep_time:.6f} seconds to maintain 50 Hz cycle rate")

if __name__ == "__main__":
    # python deploy.py
    run_onnx_model()


