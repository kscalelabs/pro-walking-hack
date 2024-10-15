"""Use onnxruntime to run a model."""

import time
from dataclasses import dataclass

import numpy as np
import onnxruntime as ort
from actuator import RobstrideMotorFeedback, RobstrideMotorsSupervisor
from berryimu import IMU, KalmanFilter

I2C_BUS = 7

@dataclass
class JointData:
    position: float = 0.0
    velocity: float = 0.0

@dataclass
class LegData:
    hip_y: JointData = JointData()
    hip_x: JointData = JointData()
    hip_z: JointData = JointData()
    knee: JointData = JointData()
    ankle_y: JointData = JointData()

    def set_feedback(self, feedback: list[RobstrideMotorFeedback]) -> None:
        self.hip_y.position = feedback[1].position
        self.hip_x.position = feedback[2].position
        self.hip_z.position = feedback[3].position
        self.knee.position = feedback[4].position
        self.ankle_y.position = feedback[5].position

        self.hip_y.velocity = feedback[1].velocity
        self.hip_x.velocity = feedback[2].velocity
        self.hip_z.velocity = feedback[3].velocity
        self.knee.velocity = feedback[4].velocity
        self.ankle_y.velocity = feedback[5].velocity

@dataclass
class ArmData:
    shoulder_y: JointData = JointData()
    shoulder_x: JointData = JointData()
    shoulder_z: JointData = JointData()
    elbow: JointData = JointData()
    wrist: JointData = JointData()

    def set_feedback(self, feedback: list[RobstrideMotorFeedback]) -> None:
        self.shoulder_y.position = feedback[1].position
        self.shoulder_x.position = feedback[2].position
        self.shoulder_z.position = feedback[3].position
        self.elbow.position = feedback[4].position
        self.wrist.position = feedback[5].position

@dataclass
class RobotData:
    left_leg: LegData = LegData()
    right_leg: LegData = LegData()
    left_arm: ArmData = ArmData()
    right_arm: ArmData = ArmData()


def run_onnx_model() -> None:
    session = ort.InferenceSession("position_control.onnx")

    # Initialize the motors.
    leg_motor_infos = {
        1: "04",
        2: "03",
        3: "03",
        4: "04",
        5: "01",
    }

    arm_motor_infos = {
        1: "03",
        2: "03",
        3: "01",
        4: "01",
        5: "01",
    }

    left_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB0", motor_infos=leg_motor_infos,
                                        target_update_rate=10000.0)

    right_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB1", motor_infos=leg_motor_infos,
                                         target_update_rate=10000.0)

    left_arm = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB2", motor_infos=arm_motor_infos,
                                         target_update_rate=10000.0)
    arm_motor_infos.pop(1)

    right_arm = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB3", motor_infos=arm_motor_infos,
                                          target_update_rate=10000.0)


    robot_data = RobotData()

    for i in range(5):
        left_leg.add_motor_to_zero(i + 1)
        time.sleep(0.01)
        right_leg.add_motor_to_zero(i + 1)
        time.sleep(0.01)
        left_arm.add_motor_to_zero(i + 1)
        time.sleep(0.01)
        if i != 0:
            right_arm.add_motor_to_zero(i + 1)
        time.sleep(0.01)

    # Initialize IMU.
    imu = IMU(I2C_BUS)
    kalman_filter = KalmanFilter(imu)

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
        vector = imu.gyr_rate()
        return np.array([vector.x, vector.y, vector.z])

    def get_euler_angles() -> np.ndarray:
        angle = kalman_filter.step()
        return np.array([angle.yaw, angle.pitch, angle.roll])

    def update_motor_data() -> None:
        left_leg_feedback = left_leg.get_latest_feedback()
        right_leg_feedback = right_leg.get_latest_feedback()
        left_arm_feedback = left_arm.get_latest_feedback()
        right_arm_feedback = right_arm.get_latest_feedback()

        robot_data.left_leg.set_feedback([left_leg_feedback[i] for i in [1, 2, 3, 4, 5]])
        robot_data.right_leg.set_feedback([right_leg_feedback[i] for i in [1, 2, 3, 4, 5]])
        robot_data.left_arm.set_feedback([left_arm_feedback[i] for i in [1, 2, 3, 4, 5]])
        robot_data.right_arm.set_feedback([right_arm_feedback[i] for i in [1, 2, 3, 4, 5]])

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
            angle = 0
            left_leg.set_position(motor_id + 1, angle)

        for motor_id, angle in enumerate(positions[5:]):
            angle = 0
            right_leg.set_position(motor_id + 1, angle)

    def run_arms() -> None:
        for motor_id, angle in enumerate(positions[:5]):
            angle = 0
            left_arm.set_position(motor_id + 1, angle)

        for motor_id, angle in enumerate(positions[:4]):
            angle = 0
            right_arm.set_position(motor_id + 2, angle)

    start_time = time.time()
    target_cycle_time = 1 / 50  # 50 Hz cycle rate
    while True:
        cycle_start_time = time.time()
        elapsed_time = cycle_start_time - start_time

        update_motor_data()

        input_data["t.1"][0] = np.float32(elapsed_time)
        input_data["dof_pos.1"] = get_joint_angles().astype(np.float32)
        input_data["dof_vel.1"] = get_joint_velocities().astype(np.float32)
        input_data["imu_ang_vel.1"] = get_angular_velocity().astype(np.float32)
        input_data["imu_euler_xyz.1"] = get_euler_angles().astype(np.float32)

        positions, actions, buffer = session.run(None, input_data)

        input_data["prev_actions.1"] = actions
        input_data["buffer.1"] = buffer

        send_positions(positions)
        # print(f"euler: {input_data['imu_euler_xyz.1']}, joint angles: {input_data['dof_pos.1']}")
        run_arms()

        cycle_end_time = time.time()
        cycle_duration = cycle_end_time - cycle_start_time
        sleep_time = max(0, target_cycle_time - cycle_duration)
        time.sleep(sleep_time)
        print(f"Slept for {sleep_time:.6f} seconds to maintain 50 Hz cycle rate")


if __name__ == "__main__":
    # python run_onnx.py
    run_onnx_model()
