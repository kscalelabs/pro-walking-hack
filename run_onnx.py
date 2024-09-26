"""Use onnxruntime to run a model."""

import numpy as np
import onnxruntime as ort


def run_onnx_model_test() -> None:
    session = ort.InferenceSession("test_model.onnx")
    input_data = np.random.randn(1, 10).astype(np.float32)
    output = session.run(None, {"x.1": input_data})
    print([o.shape for o in output])


def run_onnx_model() -> None:
    session = ort.InferenceSession("position_control.onnx")
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
    for i in range(50):
        input_data["t.1"][0] = i / 50
        torques, actions, buffer = session.run(None, input_data)
        input_data["prev_actions.1"] = actions
        input_data["buffer.1"] = buffer
        print(torques)


if __name__ == "__main__":
    # run_onnx_model_test()
    run_onnx_model()
