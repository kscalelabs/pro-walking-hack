"""Use onnxruntime to run a model."""

import time

import numpy as np
import onnxruntime as ort

from actuator import RobstrideMotorsSupervisor
from berryimu import IMU, KalmanFilter

I2C_BUS = 7

def run_onnx_model() -> None:
    session = ort.InferenceSession("position_control.onnx")

    # Initialize the motors.
    motor_infos = {
        1: {"04"},
        2: {"03"},
        3: {"03"},
        4: {"04"},
        5: {"01"},
    }

    left_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB0", motor_infos=motor_infos)
    right_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB1", motor_infos=motor_infos)

    # Initialize IMU.
    imu = IMU(I2C_BUS)
    kalman_filter = KalmanFilter(imu)

    command = {"x_vel": 0.0,
               "y_vel": 0.0,
               "rot": 0.0}

    input_data = {
        "x_vel.1": np.zeros(1).astype(np.float32),
        "y_vel.1": np.zeros(1).astype(np.float32),
        "rot.1": np.zeros(1).astype(np.float32),
        "t.1": np.zeros(1).astype(np.float32),
        "dof_pos.1": np.zeros(10).astype(np.float32),
        "dof_vel.1": np.zeros(10).astype(np.float32),
        "prev_actions.1": np.zeros(10).astype(np.float32),
        "imu_ang_vel.1": np.zeros(3).astype(np.float32),
        "imu_euler_xyz.1": np.zeros(3).astype(np.float32),
        "buffer.1": np.zeros(574).astype(np.float32),
    }

    def get_angular_velocity() -> np.ndarray:
        return imu.gyr_rate()

    def get_euler_angles() -> np.ndarray:
        return kalman_filter.step()

    def get_joint_angles() -> np.ndarray:
        angles = np.zeros(10).astype(np.float32)
        for id in range(5):
            angles[id] = left_leg.get_latest_feedback(id).position
            angles[id + 5] = right_leg.get_latest_feedback(id).position
        return angles

    def get_joint_velocities() -> np.ndarray:
        velocities = np.zeros(10).astype(np.float32)
        for id in range(5):
            velocities[id] = left_leg.get_latest_feedback(id).velocity
            velocities[id + 5] = right_leg.get_latest_feedback(id).velocity
        return velocities

    def send_positions(positions: np.ndarray) -> None:
        for motor_id, angle in enumerate(positions[:5]):
            left_leg.send_position_control(motor_id, angle)

        for motor_id, angle in enumerate(positions[5:]):
            right_leg.send_position_control(motor_id, angle)

    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time

        input_data["x_vel.1"][0] = command["x_vel"]
        input_data["y_vel.1"][0] = command["y_vel"]
        input_data["rot.1"][0] = command["rot"]

        input_data["t.1"][0] = elapsed_time
        input_data["dof_pos.1"] = get_joint_angles()
        input_data["dof_vel.1"] = get_joint_velocities()
        input_data["imu_ang_vel.1"] = get_angular_velocity()
        input_data["imu_euler_xyz.1"] = get_euler_angles()

        positions, actions, buffer = session.run(None, input_data)

        input_data["prev_actions.1"] = actions
        input_data["buffer.1"] = buffer

        send_positions(positions)
        time.sleep(1 / 50)


if __name__ == "__main__":
    # python run_onnx.py
    run_onnx_model()
