"""Record and replay joint positions for Stompy Pro."""

import time
from typing import List, Dict
import numpy as np
from actuator import RobstrideMotorsSupervisor
import json
from pathlib import Path

TARGET_RATE = 60.0 # Hz

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
        zero_on_init=True,
    )
    right_arm = RobstrideMotorsSupervisor(
        port_name=right_port,
        motor_infos=motor_config,
        target_update_rate=10000.0,
        zero_on_init=True,
    )

    # Initialize motors
    for motor_id in range(1, 6):
        left_arm.add_motor_to_zero(motor_id)
        right_arm.add_motor_to_zero(motor_id)

        left_arm.set_torque_limit(motor_id, 10.0)
        right_arm.set_torque_limit(motor_id, 10.0)

    return left_arm, right_arm

def record_positions(left_arm: RobstrideMotorsSupervisor, 
                    right_arm: RobstrideMotorsSupervisor,
                    save_file: str | None = None) -> List[Dict[str, np.ndarray]]:
    """Record positions until user presses Enter.
    
    Args:
        left_arm: Left arm supervisor
        right_arm: Right arm supervisor
        save_file: Optional path to save recorded positions
    """
    positions = [{"left": [0]*5, "right": [0]*5}]*20
    print("\nRecording started. Press Ctrl+C to stop recording...")

    for i in range(1, 5):
        left_arm.set_kp(i, 10.0)
        left_arm.set_kd(i, 1.0)
        right_arm.set_kp(i, 10.0)
        right_arm.set_kd(i, 1.0)
    
    left_arm.set_kp(5, 1.0)
    left_arm.set_kd(5, 0.1)
    right_arm.set_kp(5, 1.0)
    right_arm.set_kd(5, 0.1)

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

            for motor_id, motor_pos in enumerate(positions[-1]["left"]):
                left_arm.set_position(motor_id + 1, motor_pos)
            for motor_id, motor_pos in enumerate(positions[-1]["right"]):
                right_arm.set_position(motor_id + 1, motor_pos)

            cur_time = time.time()
            time.sleep(max(0, 1.0 / TARGET_RATE - (cur_time - last_time)))
            
    except KeyboardInterrupt:
        print("\nRecording interrupted.")

    print(f"Recording stopped. Captured {len(positions)} frames.")
    
    if save_file:
        json_positions = [
            {
                "left": pos["left"].tolist(),
                "right": pos["right"].tolist()
            } for pos in positions
        ]
        
        save_data = {
            "target_rate": TARGET_RATE,
            "positions": json_positions
        }
        
        Path(save_file).parent.mkdir(parents=True, exist_ok=True)
        with open(save_file, 'w') as f:
            json.dump(save_data, f, indent=2)
        print(f"Saved recording to {save_file}")
        
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
                set_time = time.time()
                left_arm.set_position(i + 1, pos["left"][i])
                # print(f"Set left arm {i + 1} in {time.time() - set_time} seconds")
                set_time = time.time()
                right_arm.set_position(i + 1, pos["right"][i])
                # print(f"Set right arm {i + 1} in {time.time() - set_time} seconds")
            
            sleep_time = max(0, 1.0 / TARGET_RATE - (time.time() - start_time))
            # print(f"Sleeping for {sleep_time} ({time.time() - start_time}) seconds")
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
        raise

def main():
    print("Initializing robot...")
    left_arm, right_arm = initialize_robot()
    recorded_positions = None
    
    try:
        while True:
            if recorded_positions is None:
                save_path = input("Enter path to save recording (or press Enter to skip saving): ").strip()
                save_file = save_path if save_path else None
                
                input("Press Enter to start recording...")
                recorded_positions = record_positions(left_arm, right_arm, save_file)

                print("Returning to home position...")

                # Set intermediate PD gains
                for motor_id in range(1, 3):
                    left_arm.set_kp(motor_id, 10.0)
                    right_arm.set_kp(motor_id, 10.0)
                    left_arm.set_kd(motor_id, 2.0)
                    right_arm.set_kd(motor_id, 2.0)
                
                for motor_id in range(3, 6):
                    left_arm.set_kp(motor_id, 10.0)
                    right_arm.set_kp(motor_id, 10.0)
                    left_arm.set_kd(motor_id, 2.0)
                    right_arm.set_kd(motor_id, 2.0)
                
                time.sleep(1.0)

                # Set PD gains
                for motor_id in range(1, 3):
                    left_arm.set_kp(motor_id, 120.0)
                    right_arm.set_kp(motor_id, 120.0)
                    left_arm.set_kd(motor_id, 5.0)
                    right_arm.set_kd(motor_id, 5.0)
                
                for motor_id in range(3, 6):
                    left_arm.set_kp(motor_id, 30.0)
                    right_arm.set_kp(motor_id, 30.0)
                    left_arm.set_kd(motor_id, 2.0)
                    right_arm.set_kd(motor_id, 2.0)

            else:
                input("Press Enter to replay motion (or Ctrl+C to exit)...")
                replay_positions(recorded_positions, left_arm, right_arm)
    
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
