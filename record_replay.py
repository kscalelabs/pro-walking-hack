"""Record and replay joint positions for Stompy Pro."""

import time
from typing import List, Dict
import numpy as np
from actuator import RobstrideMotorsSupervisor

TARGET_RATE = 20.0 # Hz

# Reference motor configuration from vr_teleop.py
motor_config = {
    1: "03",
    2: "03",
    3: "01",
    4: "01",
    5: "01",
}

def initialize_robot() -> tuple[RobstrideMotorsSupervisor, RobstrideMotorsSupervisor]:
    """Initialize both arms of the robot."""
    left_port = "/dev/ttyCH341USB1"
    right_port = "/dev/ttyCH341USB0"

    left_arm = RobstrideMotorsSupervisor(
        port_name=left_port,
        motor_infos=motor_config,
        target_update_rate=10000.0,
    )
    right_arm = RobstrideMotorsSupervisor(
        port_name=right_port,
        motor_infos=motor_config,
        target_update_rate=10000.0,
    )

    # Initialize motors
    for motor_id in range(1, 6):
        left_arm.add_motor_to_zero(motor_id)
        right_arm.add_motor_to_zero(motor_id)

    return left_arm, right_arm

def record_positions(left_arm: RobstrideMotorsSupervisor, 
                    right_arm: RobstrideMotorsSupervisor) -> List[Dict[str, np.ndarray]]:
    """Record positions until user presses Enter."""
    positions = []
    print("\nRecording started. Press Ctrl+C to stop recording...")

    for i in range(1, 6):
        left_arm.set_kp(i, 0.0)
        left_arm.set_kd(i, 0.0)
        right_arm.set_kp(i, 0.0)
        right_arm.set_kd(i, 0.0)

    try:
        while True:
            last_time = time.time()

            # Get current positions
            left_feedback = left_arm.get_latest_feedback()
            right_feedback = right_arm.get_latest_feedback()
            
            current_pos = {
                "left": np.array([left_feedback[i].position for i in range(1, 6)]),
                "right": np.array([right_feedback[i].position for i in range(1, 6)])
            }
            positions.append(current_pos)

            cur_time = time.time()
            time.sleep(max(0, 1.0 / TARGET_RATE - (cur_time - last_time)))  # Record at 50Hz
            
    except KeyboardInterrupt:
        print("\nRecording interrupted.")
    
    print(f"Recording stopped. Captured {len(positions)} frames.")
    print(positions)
    return positions

def replay_positions(positions: List[Dict[str, np.ndarray]], 
                    left_arm: RobstrideMotorsSupervisor, 
                    right_arm: RobstrideMotorsSupervisor) -> None:
    """Replay recorded positions."""
    print("\nReplaying motion... Press Ctrl+C to stop.")
    
    try:
        for pos in positions:
            start_time = time.time()
            # Set positions for both arms
            for i in range(5):
                left_arm.set_position(i + 1, pos["left"][i])
                right_arm.set_position(i + 1, pos["right"][i])
            
            sleep_time = max(0, 1.0 / TARGET_RATE - (time.time() - start_time))
            # print(f"Sleeping for {sleep_time} ({time.time() - start_time}) seconds")
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nReplay interrupted.")

def main():
    print("Initializing robot...")
    left_arm, right_arm = initialize_robot()
    recorded_positions = None
    
    try:
        while True:
            if recorded_positions is None:
                input("Press Enter to start recording...")
                recorded_positions = record_positions(left_arm, right_arm)

                print("Recorded positions")

                # Set PD gains
                for motor_id in range(1, 3):
                    left_arm.set_kp(motor_id, 60.0)
                    right_arm.set_kp(motor_id, 60.0)
                    left_arm.set_kd(motor_id, 5.0)
                    right_arm.set_kd(motor_id, 5.0)
                
                for motor_id in range(3, 6):
                    left_arm.set_kp(motor_id, 10.0)
                    right_arm.set_kp(motor_id, 10.0)
                    left_arm.set_kd(motor_id, 0.1)
                    right_arm.set_kd(motor_id, 0.1)

            else:
                input("Press Enter to replay motion (or Ctrl+C to exit)...")
                replay_positions(recorded_positions, left_arm, right_arm)
    
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
