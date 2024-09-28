"""Use onnxruntime to run a model."""

import time

import numpy as np
import onnxruntime as ort


def run_onnx_model() -> None:
    session = ort.InferenceSession("position_control.onnx")

    input_data = {
        "x_vel.1": np.zeros(1).astype(np.float32),  # 0
        "y_vel.1": np.zeros(1).astype(np.float32),  # 0
        "rot.1": np.zeros(1).astype(np.float32),  # 0
        "t.1": np.zeros(1).astype(np.float32),  # 自开始以来的时间，以秒为单位
        "dof_pos.1": np.zeros(10).astype(np.float32),  # 关节角位置
        "dof_vel.1": np.zeros(10).astype(np.float32),  # 关节角速度
        "prev_actions.1": np.zeros(10).astype(np.float32),  # 从一步到下一步，从零开始
        "imu_ang_vel.1": np.zeros(3).astype(np.float32),  # 0
        "imu_euler_xyz.1": np.zeros(3).astype(np.float32),  # 0
        "buffer.1": np.zeros(574).astype(np.float32),  # 从一步到下一步，从零开始
    }

    def get_joint_angles() -> np.ndarray:
        # 获取执行器当前的关节角度
        return np.zeros(10).astype(np.float32)

    def get_joint_velocities() -> np.ndarray:
        # 获取执行器的当前角速度
        return np.zeros(10).astype(np.float32)

    def send_torques(torques: np.ndarray) -> None:
        # 将扭矩发送到电机
        print(torques)

    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        input_data["t.1"][0] = elapsed_time  # 自开始以来的时间，以秒为单位
        input_data["dof_pos.1"] = get_joint_angles()
        input_data["dof_vel.1"] = get_joint_velocities()
        torques, actions, buffer = session.run(None, input_data)
        input_data["prev_actions.1"] = actions
        input_data["buffer.1"] = buffer
        send_torques(torques)
        time.sleep(1 / 50)  # 暂停运行 20 毫秒


if __name__ == "__main__":
    # python run_onnx.py
    run_onnx_model()
