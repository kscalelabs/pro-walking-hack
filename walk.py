import time
import pykos
import numpy as np
from kinfer.inference import ONNXModel

class RealPPOController:
    def __init__(self):
        self.kos = pykos.KOS()
        self.kinfer = ONNXModel("model.onnx")
        
        # Get model metadata
        metadata = self.kinfer.get_metadata()
        self.model_info = {
            "num_actions": metadata["num_actions"],
            "num_observations": metadata["num_observations"],
            "robot_effort": metadata["robot_effort"],
            "robot_stiffness": metadata["robot_stiffness"],
            "robot_damping": metadata["robot_damping"],
        }
        
        # Initialize input state with dynamic sizes from metadata
        self.input_data = {
            "x_vel.1": np.zeros(1, dtype=np.float32),
            "y_vel.1": np.zeros(1, dtype=np.float32),
            "rot.1": np.zeros(1, dtype=np.float32),
            "t.1": np.zeros(1, dtype=np.float32),
            "dof_pos.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "dof_vel.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "prev_actions.1": np.zeros(self.model_info["num_actions"], dtype=np.float32),
            "imu_ang_vel.1": np.zeros(3, dtype=np.float32),
            "imu_euler_xyz.1": np.zeros(3, dtype=np.float32),
            "buffer.1": np.zeros(self.model_info["num_observations"], dtype=np.float32),
        }
        
        # Track previous actions and buffer for recurrent state
        self.actions = np.zeros(self.model_info["num_actions"], dtype=np.float32)
        self.buffer = np.zeros(self.model_info["num_observations"], dtype=np.float32)
        
        # Walking command defaults
        self.command = {
            "x_vel": 0.4,
            "y_vel": 0.0,
            "rot": 0.0,
        }

    def update_robot_state(self):
        """Update input data from robot sensors"""
        motor_feedback = self.kos.get_actuator_states()

        breakpoint()

        # Create dictionary of motor feedback to motor id
        motor_feedback_dict = {
            motor["id"]: motor for motor in motor_feedback
        }

        # Check that each motor is enabled
        for motor in motor_feedback:
            if not motor["enabled"]:
                raise RuntimeError(f"Motor {motor['name']} is not enabled")

        # Should be arranged left to right, top to bottom
        joint_positions = np.array([
            motor["position"] for motor in motor_feedback
        ])

        joint_velocities = np.array([
            motor["velocity"] for motor in motor_feedback
        ])
        
        imu_ang_vel = np.zeros(3)  # Replace with actual IMU angular velocity
        imu_euler = np.zeros(3)    # Replace with actual IMU orientation

        # Update input dictionary
        self.input_data["dof_pos.1"] = joint_positions.astype(np.float32)
        self.input_data["dof_vel.1"] = joint_velocities.astype(np.float32)
        self.input_data["imu_ang_vel.1"] = imu_ang_vel.astype(np.float32)
        self.input_data["imu_euler_xyz.1"] = imu_euler.astype(np.float32)
        self.input_data["prev_actions.1"] = self.actions
        self.input_data["buffer.1"] = self.buffer

    def move_actuators(self, positions):
        """Move actuators to desired positions"""
        left_positions = positions[:5]
        right_positions = positions[5:]

        # Send positions to robot

    def step(self, dt):
        """Run one control step"""
        # Update command velocities
        self.input_data["x_vel.1"][0] = np.float32(self.command["x_vel"])
        self.input_data["y_vel.1"][0] = np.float32(self.command["y_vel"])
        self.input_data["rot.1"][0] = np.float32(self.command["rot"])
        self.input_data["t.1"][0] = np.float32(dt)

        # Update robot state
        self.update_robot_state()

        # Run inference
        outputs = self.kinfer(self.input_data)

        # Extract outputs
        positions = outputs["actions_scaled"]
        self.actions = outputs["actions"]
        self.buffer = outputs["x.3"]

        # Clip positions for safety
        positions = np.clip(positions, -0.5, 0.5)

        # Send positions to robot
        self.move_actuators(positions)
        return positions


def main():
    controller = RealPPOController()
    dt = 0.01 # 100Hz
    while True:
        loop_start_time = time.time()
        controller.step(dt)
        loop_end_time = time.time()
        sleep_time = max(0, dt - (loop_end_time - loop_start_time))
        time.sleep(sleep_time)

if __name__ == "__main__":
    main()
