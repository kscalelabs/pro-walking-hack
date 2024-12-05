""" Whoever is reading this, forgive us.

TODO: check assumptions:
# 1. Check if torque values throguh position contorl are the same - 
# if not, we should pass straight torque value
# 2. 

"""
import time
import pykos
import numpy as np
from kinfer.inference import ONNXModel
from imu import HexmoveImuReader


class RealPPOController:
    def __init__(self, model_path: str, signs: np.ndarray):
        self.kos = pykos.KOS()
        self.kinfer = ONNXModel(model_path)

        self.signs = signs

        # Walking command defaults
        self.command = {
            "x_vel": 0.4,
            "y_vel": 0.0,
            "rot": 0.0,
        }

        # Get model metadata
        metadata = self.kinfer.get_metadata()
        self.model_info = {
            "num_actions": metadata["num_actions"],
            "num_observations": metadata["num_observations"],
            "robot_effort": metadata["robot_effort"],
            "robot_stiffness": metadata["robot_stiffness"],
            "robot_damping": metadata["robot_damping"],
            "default_standing": metadata["default_standing"],
        }
        
        # Add IMU initialization
        self.imu_reader = HexmoveImuReader("can0", 1, 1)
        self.euler_signs = np.array([-1, -1, -1])

        self.left_arm_ids = [11, 12, 13, 14, 15]#, 16]
        self.right_arm_ids = [21, 22, 23, 24, 25]#, 26]
        self.left_leg_ids = [31, 32, 33, 34, 35]
        self.right_leg_ids = [41, 42, 43, 44, 45]

        self.type_four_ids = [limb[id] for limb in [self.left_leg_ids, self.right_leg_ids] for id in [0, 3]]
        self.type_three_ids = [limb[id] for limb in [self.left_leg_ids, self.right_leg_ids] for id in [1, 2]]
        self.type_two_ids = [limb[id] for limb in [self.left_leg_ids, self.right_leg_ids] for id in [4]]

        self.all_ids = self.left_arm_ids + self.right_arm_ids + self.left_leg_ids + self.right_leg_ids

        # Configure all motors
        for id in self.type_four_ids:
            self.kos.actuator.configure_actuator(actuator_id=id, kp=120, kd=10, torque_enabled=True)

        for id in self.type_three_ids:
            self.kos.actuator.configure_actuator(actuator_id=id, kp=60, kd=5, torque_enabled=True)

        for id in self.type_two_ids:
            self.kos.actuator.configure_actuator(actuator_id=id, kp=17, kd=5, torque_enabled=True)

        # Calculate initial IMU offset as running average over 5 seconds
        num_samples = 50  # 10 Hz for 5 seconds
        angles = []
        print("Calculating IMU offset...")
        for _ in range(num_samples):
            imu_data = self.imu_reader.get_data()
            angles.append([imu_data.x_angle, imu_data.y_angle, imu_data.z_angle])
            time.sleep(0.1)

        self.angular_offset = np.mean(angles, axis=0)
        print(f"IMU offset calculated: {self.angular_offset}")

        # Negate default standing offsets because sim is ccw+ and real is cw+
        self.left_offsets = self.signs[:5] * np.array(self.model_info["default_standing"][:5])

        # TODO - make the mapping again for each motor idependenly
        self.right_offsets = self.signs[5:] * np.array(self.model_info["default_standing"][5:])

        self.offsets = np.concatenate([self.left_offsets, self.right_offsets])

        print(f"Offsets: {self.offsets}")
        
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

    def update_robot_state(self):
        """Update input data from robot sensors"""        
        # Get IMU data
        imu_data = self.imu_reader.get_data()
        
        # Calculate IMU angular velocity
        imu_ang_vel = np.array([
            imu_data.x_velocity, 
            imu_data.y_velocity, 
            imu_data.z_velocity
        ]) * self.euler_signs
        
        # Calculate IMU orientation (euler angles)
        angles = np.deg2rad([
            imu_data.x_angle,
            imu_data.y_angle, 
            imu_data.z_angle
        ] - self.angular_offset) * self.euler_signs
        
        # Normalize angles to [-pi, pi]
        angles[angles > np.pi] -= 2 * np.pi
        angles[angles < -np.pi] += 2 * np.pi

        motor_feedback = self.kos.actuator.get_actuators_state(self.all_ids)

        # Create dictionary of motor feedback to motor id
        self.motor_feedback_dict = {
            motor.actuator_id: motor for motor in motor_feedback
        }

        # Check that each motor is enabled
        for motor in self.motor_feedback_dict.values():
            if not motor.online and motor.actuator_id in self.left_leg_ids + self.right_leg_ids:
                raise RuntimeError(f"Motor {motor.actuator_id} is not online")
        
        # Should be arranged left to right, top to bottom
        joint_positions = np.concatenate([
            np.array([self.motor_feedback_dict[id].position for id in self.left_leg_ids]),
            np.array([self.motor_feedback_dict[id].position for id in self.right_leg_ids])
        ])

        joint_velocities = np.concatenate([
            np.array([self.motor_feedback_dict[id].velocity for id in self.left_leg_ids]),
            np.array([self.motor_feedback_dict[id].velocity for id in self.right_leg_ids])
        ])

        joint_positions = np.deg2rad(joint_positions)

        joint_velocities = np.deg2rad(joint_velocities)


        joint_positions -= self.offsets


        # Multiply positions and velocities by the sign mapping of the motor
        joint_positions = self.signs * joint_positions
        joint_velocities = self.signs * joint_velocities

        # Update input dictionary
        self.input_data["dof_pos.1"] = joint_positions.astype(np.float32)
        self.input_data["dof_vel.1"] = joint_velocities.astype(np.float32)
        self.input_data["imu_ang_vel.1"] = imu_ang_vel.astype(np.float32)
        self.input_data["imu_euler_xyz.1"] = angles.astype(np.float32)
        self.input_data["prev_actions.1"] = self.actions
        self.input_data["buffer.1"] = self.buffer

    def move_actuators(self, positions):
        """Move actuators to desired positions"""
        left_positions = positions[:5]
        right_positions = positions[5:]

        actuator_commands = []

        for id, position in zip(self.left_leg_ids, left_positions):
            actuator_commands.append({"actuator_id": id, "position": position})

        for id, position in zip(self.right_leg_ids, right_positions):
            actuator_commands.append({"actuator_id": id, "position": position})

        self.kos.actuator.command_actuators(actuator_commands)

    def set_default_position(self):
        """Set the robot to the default position"""
        self.move_actuators(np.rad2deg(self.offsets))

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
        positions = np.clip(positions, -0.75, 0.75)

        expected_positions = self.signs * positions + self.offsets
        expected_positions = np.rad2deg(expected_positions)

        # Calculate position error (actual - desired)
        current_positions = np.concatenate([
            np.array([self.motor_feedback_dict[id].position for id in self.left_leg_ids]),
            np.array([self.motor_feedback_dict[id].position for id in self.right_leg_ids])
        ])
        position_error = current_positions - expected_positions
        print(f"Position error (deg): {position_error}")

        # Send positions to robot
        self.move_actuators(expected_positions)
        return positions


def main():
    controller = RealPPOController(model_path="gpr_walking.kinfer")
    frequency = 1/100. # 100Hz
    # dt = 0.1 # Slow frequency for debugging
    start_time = time.time()
    try:
        while True:
            loop_start_time = time.time()
            controller.step(start_time)
            loop_end_time = time.time()
            sleep_time = max(0, frequency - (loop_end_time - loop_start_time))
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        controller.kos.close()


if __name__ == "__main__":
    main()
