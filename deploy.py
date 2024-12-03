"""Use onnxruntime to run a model."""

import threading
import time
from dataclasses import dataclass, field

import numpy as np
import onnxruntime as ort
from actuator import RobstrideMotorFeedback, RobstrideMotorsSupervisor
from imu import HexmoveImuReader
from kinfer.inference import ONNXModel

PD_CONSTANTS = {
    1: {
        "name": "hip_y",
        "p": 250.0,
        "d": 30.0,
    },
    2: {
        "name": "hip_x", 
        "p": 125.0,
        "d": 15.0,
    },
    3: {
        "name": "hip_z",
        "p": 150.0,
        "d": 15.0,
    },
    4: {
        "name": "knee",
        "p": 200.0,
        "d": 15.0,
    },
    5: {
        "name": "ankle_y",
        "p": 150.0,
        "d": 20.0,
    },
}

# PD_CONSTANTS = {
#     1: {
#         "name": "hip_y",
#         "p": 100.0,
#         "d": 10.0,
#     },
#     2: {
#         "name": "hip_x",
#         "p": 80.0,
#         "d": 10.0,
#     },
#     3: {
#         "name": "hip_z",
#         "p": 80.0,
#         "d": 10.0,
#     },
#     4: {
#         "name": "knee",
#         "p": 100.0,
#         "d": 10.0,
#     },
#     5: {
#         "name": "ankle_y",
#         "p": 80.0,
#         "d": 10.0,
#     },
# }

MOTOR_STARTUP_SAFETY_BUFFER = 1.5

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
    kinfer = ONNXModel("walking_pro_kinfer.onnx")

    # Initialize the motors.
    left_leg_motor_infos = {
        21: "04",
        22: "03",
        23: "03",
        24: "04",
        25: "02",
    }

    right_leg_motor_infos = {
        31: "04",
        32: "03",
        33: "03",
        34: "04",
        35: "02",
    }

    # Initialize CAN-based IMU
    """
       sudo ip link set can0 up type can bitrate 500000
       ip link 
    """
    imu_reader = HexmoveImuReader("can0", 1, 1)

    # Converting from real to sim
    euler_signs = np.array([-1, -1, -1])

    time.sleep(1)

    imu_data = imu_reader.get_data()

    # IMU offset
    angular_offset = np.array([imu_data.x_angle, imu_data.y_angle, imu_data.z_angle])
    print(f"angular_offset: {angular_offset}")
    imu_data = imu_reader.get_data()
    print(f"imu_data: {imu_data.x_angle - angular_offset[0]}, {imu_data.y_angle - angular_offset[1]}, {imu_data.z_angle - angular_offset[2]}")
    left_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB2", motor_infos=left_leg_motor_infos,
                                        target_update_rate=10000.0)

    right_leg = RobstrideMotorsSupervisor(port_name="/dev/ttyCH341USB3", motor_infos=right_leg_motor_infos,
                                         target_update_rate=10000.0)

    robot_data = RobotData()

    # Reference offsets for standing
    standing_offset = RobotData()

    # # Leg 1 (Values from robstride)
    # standing_offset.right_leg.hip_y.position = 1.57
    # standing_offset.right_leg.hip_x.position = 0.384
    # standing_offset.right_leg.hip_z.position = 1.57
    # standing_offset.right_leg.knee.position = 2.17 
    # standing_offset.right_leg.ankle_y.position = 0.7

    # # Leg 2
    # standing_offset.left_leg.hip_y.position = 4.71
    # standing_offset.left_leg.hip_x.position = 5.896
    # standing_offset.left_leg.hip_z.position = 4.71
    # standing_offset.left_leg.knee.position = 4.11
    # standing_offset.left_leg.ankle_y.position = 5.58

    # # Leg 1 (OLD BEFORE BEING STRAIGED UP MIRRED)
    # standing_offset.right_leg.hip_y.position = 1.203 #1.01
    # standing_offset.right_leg.hip_x.position = 0.384
    # standing_offset.right_leg.hip_z.position = 1.57
    # standing_offset.right_leg.knee.position = 1.453 #1.3
    # standing_offset.right_leg.ankle_y.position = 1.033 #1.02

    # Leg 2
    standing_offset.left_leg.hip_y.position = 5.08
    standing_offset.left_leg.hip_x.position = 5.896
    standing_offset.left_leg.hip_z.position = 4.71
    standing_offset.left_leg.knee.position = 4.83 - 0.3
    standing_offset.left_leg.ankle_y.position = 5.25 + 0.3

    # Leg 1
    standing_offset.right_leg.hip_y.position = 2 * np.pi - standing_offset.left_leg.hip_y.position
    standing_offset.right_leg.hip_x.position = 2 * np.pi - standing_offset.left_leg.hip_x.position
    standing_offset.right_leg.hip_z.position = 2 * np.pi - standing_offset.left_leg.hip_z.position
    standing_offset.right_leg.knee.position = 2 * np.pi - standing_offset.left_leg.knee.position
    standing_offset.right_leg.ankle_y.position = 2 * np.pi - standing_offset.left_leg.ankle_y.position

    # for i in range(5):
    #     left_leg.add_motor_to_zero(i + 1)
    #     right_leg.add_motor_to_zero(i + 1)
    #     time.sleep(0.01)
    time.sleep(1)
    left_positions = left_leg.get_latest_feedback()
    right_positions = right_leg.get_latest_feedback()

    print(f"Left positions: {left_positions}")
    print(f"Right positions: {right_positions}")

    for motor_id, _ in leg_motor_infos.items():
        # Check if starting positions are valid and raise an error if not
        assert left_positions[motor_id].position > MOTOR_STARTUP_SAFETY_BUFFER # leg 2
        assert right_positions[motor_id].position < 2*np.pi - MOTOR_STARTUP_SAFETY_BUFFER # leg 1
        # if left_positions[motor_id].position < MOTOR_STARTUP_SAFETY_BUFFER: # leg 2
        #     raise RuntimeError(f"Left leg motor {motor_id} is too close to zero: {left_positions[motor_id].position}")
        
        # if right_positions[motor_id].position > 2*np.pi - MOTOR_STARTUP_SAFETY_BUFFER: # leg 1
        #     raise RuntimeError(f"Right leg motor {motor_id} is too close to zero: {right_positions[motor_id].position}")    
        time.sleep(0.01)

    
    # VERY IMPORTANT TO DO THIS
    for id, feedback in left_positions.items():
        if feedback.mode == "Motor":
            if feedback.position > 10:
                print("ERROR")
                exit()
            left_leg.set_position(id, feedback.position)
            time.sleep(0.01)
        else:
            raise RuntimeError(f"Left leg motor {id} is not in Motor mode: {feedback}")

    for id, feedback in right_positions.items():
        if feedback.mode == "Motor":
            right_leg.set_position(id, feedback.position)
            time.sleep(0.01)
        else:
            print(f"Right leg motor {id} is not in Motor mode: {feedback}")
            raise RuntimeError(f"Right leg motor {id} is not in Motor mode: {feedback}")
    
    time.sleep(0.5)

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

    command = {"x_vel": 0.4,
               "y_vel": 0.0,
               "rot": 0.0,}

    input_data["x_vel.1"][0] = np.float32(command["x_vel"])
    input_data["y_vel.1"][0] = np.float32(command["y_vel"])
    input_data["rot.1"][0] = np.float32(command["rot"])

    def get_angular_velocity() -> np.ndarray:
        # return np.array([imu_data.x_velocity, imu_data.y_velocity, imu_data.z_velocity]) * euler_signs
        return np.zeros(3, dtype=np.float32)

    def get_euler_angles() -> np.ndarray:
        # Roll, pitch, yaw
        angles =  np.deg2rad([imu_data.x_angle, imu_data.y_angle, imu_data.z_angle] - angular_offset) * euler_signs
        angles[angles > np.pi] -= 2 * np.pi
        angles[angles < -np.pi] += 2 * np.pi
        return np.zeros(3, dtype=np.float32)
        return angles


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
        real_joints = np.array([
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

        offset_array = np.array([
            standing_offset.left_leg.hip_y.position,
            standing_offset.left_leg.hip_x.position,
            standing_offset.left_leg.hip_z.position,
            standing_offset.left_leg.knee.position,
            standing_offset.left_leg.ankle_y.position,
            standing_offset.right_leg.hip_y.position,
            standing_offset.right_leg.hip_x.position,
            standing_offset.right_leg.hip_z.position,
            standing_offset.right_leg.knee.position,
            standing_offset.right_leg.ankle_y.position,
        ])

        return (real_joints - offset_array) * -1

    def get_joint_velocities() -> np.ndarray:
        real_joints = np.array([
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

        return (real_joints) * -1

    def send_positions(positions: np.ndarray) -> None:

        MAX_DELTA_POSITION = 0.5

        left_leg_feedback = left_leg.get_latest_feedback()
        right_leg_feedback = right_leg.get_latest_feedback()

        offset_array = np.array([
            standing_offset.left_leg.hip_y.position,
            standing_offset.left_leg.hip_x.position,
            standing_offset.left_leg.hip_z.position,
            standing_offset.left_leg.knee.position,
            standing_offset.left_leg.ankle_y.position,
        ])
        for motor_id, angle in enumerate(positions[:5] + offset_array[:5]):
            if abs(left_leg_feedback[motor_id + 1].position - angle) < MAX_DELTA_POSITION:
                left_leg.set_position(motor_id + 1, angle)
            else:
                print(f"Left leg motor {motor_id + 1} is too far from target position: {left_leg_feedback[motor_id + 1].position} -> {angle}")
        
        offset_array = np.array([
            standing_offset.right_leg.hip_y.position,
            standing_offset.right_leg.hip_x.position,
            standing_offset.right_leg.hip_z.position,
            standing_offset.right_leg.knee.position,
            standing_offset.right_leg.ankle_y.position,
        ])

        for motor_id, angle in enumerate(positions[5:] + offset_array[:5]):
            if abs(right_leg_feedback[motor_id + 1].position - angle) < MAX_DELTA_POSITION:
                right_leg.set_position(motor_id + 1, angle)
            else:
                print(f"Right leg motor {motor_id + 1} is too far from target position: {right_leg_feedback[motor_id + 1].position} -> {angle}")

    start_time = time.time()
    target_cycle_time = 1.0 / 100.0  # 100 Hz cycle rate

    actions = np.zeros(10, dtype=np.float32)
    buffer = np.zeros(615, dtype=np.float32)

    left_ref = [
        standing_offset.left_leg.hip_y.position,
        standing_offset.left_leg.hip_x.position,
        standing_offset.left_leg.hip_z.position,
        standing_offset.left_leg.knee.position,
        standing_offset.left_leg.ankle_y.position,
    ]

    right_ref = [
        standing_offset.right_leg.hip_y.position,
        standing_offset.right_leg.hip_x.position,
        standing_offset.right_leg.hip_z.position,
        standing_offset.right_leg.knee.position,
        standing_offset.right_leg.ankle_y.position,
    ]

    # while True:
    #     for motor_id in [1, 2, 3, 4, 5]:
    #         left_leg.set_position(motor_id, left_ref[motor_id - 1])
    #         right_leg.set_position(motor_id, right_ref[motor_id - 1])
    #         # print(f"Trying to set left leg motor {motor_id} to {left_ref[motor_id - 1]}")
    #         # print(f"Trying to set right leg motor {motor_id} to {right_ref[motor_id - 1]}")
    #         time.sleep(0.01)
    #     time.sleep(0.5)
    
    # exit()

    while True:
        cycle_start_time = time.time()
        elapsed_time = cycle_start_time - start_time

        update_motor_data()
        imu_data = imu_reader.get_data()

        input_data["t.1"][0] = np.float32(elapsed_time).astype(np.float32)
        input_data["dof_pos.1"] = get_joint_angles().astype(np.float32)
        input_data["dof_vel.1"] = get_joint_velocities().astype(np.float32)
        input_data["imu_ang_vel.1"] = get_angular_velocity().astype(np.float32)
        input_data["imu_euler_xyz.1"] = get_euler_angles().astype(np.float32)

        input_data["prev_actions.1"] = actions.astype(np.float32)
        input_data["buffer.1"] = buffer.astype(np.float32)

        outputs = kinfer(input_data)

        positions = outputs["actions_scaled"]
        actions = outputs["actions"]
        buffer = outputs["x.3"]

        positions = positions

        if np.any(positions > 0.5) or np.any(positions < -0.5):
            print(f"Positions out of bounds: {positions}")
            positions = np.clip(positions, -0.5, 0.5) 
            print(f"Dof_pos: {input_data['dof_pos.1']}")
#         print(positions)
#         positions = np.zeros(10, dtype=np.float32) - 0.1
#         positions = np.array([-0.07445415,  0.03433557, -0.03165408,  0.03360979, -0.13386671,  0.14077598,
#   0.08295336,  0.00957936,  0.11282689, -0.1593551 ])
        send_positions(positions)
        # print(robot_data)
        # print(f"imu_ang_euler: {input_data['imu_euler_xyz.1']}")

        cycle_end_time = time.time()
        cycle_duration = cycle_end_time - cycle_start_time
        sleep_time = max(0, target_cycle_time - cycle_duration)
        time.sleep(sleep_time)
        # print(f"Slept for {sleep_time:.6f} seconds to maintain 50 Hz cycle rate")

if __name__ == "__main__":
    # python deploy.py
    run_onnx_model()


