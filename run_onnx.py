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

    def get_joint_angles() -> np.ndarray:
        angles = np.zeros(10).astype(np.float32)
        left_leg_feedback = left_leg.get_latest_feedback()
        right_leg_feedback = right_leg.get_latest_feedback()

        if len(left_leg_feedback) == 0 or len(right_leg_feedback) == 0:
            return angles
        

        # For testing
        left_arm_feedback = left_arm.get_latest_feedback()
        right_arm_feedback = right_arm.get_latest_feedback()

        for id in range(5):
            angles[id] = left_leg_feedback[id + 1].position
            angles[id + 5] = right_leg_feedback[id + 1].position
        return angles

    def get_joint_velocities() -> np.ndarray:
        velocities = np.zeros(10).astype(np.float32)
        left_leg_feedback = left_leg.get_latest_feedback()
        right_leg_feedback = right_leg.get_latest_feedback()

        if len(left_leg_feedback) == 0 or len(right_leg_feedback) == 0:
            return velocities

        for id in range(5):
            velocities[id] = left_leg_feedback[id + 1].velocity
            velocities[id + 5] = right_leg_feedback[id + 1].velocity
        return velocities

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
