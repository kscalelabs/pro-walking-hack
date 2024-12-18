"""
Simple script to control kbot motors using keyboard inputs:
- Up arrow: increase position by n degrees
- Down arrow: decrease position by n degrees
- Esc: quit
"""

import pykos
import keyboard
import traceback
import time

# Global state for key tracking
key_pressed = None

# Configuration
STEP_SIZE = 10.0  # Default step size in degrees
MAX_STEP = 30.0  # Maximum allowed step size in degrees

J_NAME_TO_ID = {
    # right arm
    "right_arm_shoulder_pitch": 21,
    "right_arm_shoulder_roll": 22,
    "right_arm_shoulder_yaw": 23,
    "right_arm_elbow_pitch": 24,
    "right_arm_elbow_roll": 25,
    # left arm
    "left_arm_shoulder_pitch": 11,
    "left_arm_shoulder_roll": 12,
    "left_arm_shoulder_yaw": 13,
    "left_arm_elbow_pitch": 14,
    "left_arm_elbow_roll": 15,
    # right leg
    "right_leg_hip_pitch": 41,
    "right_leg_hip_roll": 42,
    "right_leg_hip_yaw": 43,
    "right_leg_knee_pitch": 44,
    "right_leg_ankle_pitch": 45,
    # left leg
    "left_leg_hip_pitch": 31,
    "left_leg_hip_roll": 32,
    "left_leg_hip_yaw": 33,
    "left_leg_knee_pitch": 34,
    "left_leg_ankle_pitch": 35,
}

J_ID_TO_NAME = {value: key for key, value in J_NAME_TO_ID.items()}


def configure_actuators(kos, actuator_ids, kp, kd, max_torque):
    if not actuator_ids:
        return
    for actuator_id in actuator_ids:
        try:
            print(f"Configuring actuator {actuator_id} - {J_ID_TO_NAME[actuator_id]}")
            kos.actuator.configure_actuator(
                actuator_id=actuator_id, kp=kp, kd=kd, max_torque=max_torque, torque_enabled=True
            )
        except Exception as e:
            print(f"Failed to configure actuator {actuator_id} - {J_ID_TO_NAME[actuator_id]}: {e}")


def set_up(kos):
    # Define actuator types based on joint IDs

    # Leg types
    leg_type_four_ids = [
        J_NAME_TO_ID["left_leg_hip_pitch"],
        J_NAME_TO_ID["left_leg_knee_pitch"],
        J_NAME_TO_ID["right_leg_hip_pitch"],
        J_NAME_TO_ID["right_leg_knee_pitch"],
    ]
    leg_type_three_ids = [
        J_NAME_TO_ID["left_leg_hip_roll"],
        J_NAME_TO_ID["left_leg_hip_yaw"],
        J_NAME_TO_ID["right_leg_hip_roll"],
        J_NAME_TO_ID["right_leg_hip_yaw"],
    ]
    leg_type_two_ids = [J_NAME_TO_ID["left_leg_ankle_pitch"], J_NAME_TO_ID["right_leg_ankle_pitch"]]

    # Arm types
    arm_type_three_ids = [
        J_NAME_TO_ID["left_arm_shoulder_pitch"],
        J_NAME_TO_ID["left_arm_shoulder_roll"],
        J_NAME_TO_ID["right_arm_shoulder_pitch"],
        J_NAME_TO_ID["right_arm_shoulder_roll"],
    ]
    arm_type_two_ids = [
        J_NAME_TO_ID["left_arm_shoulder_yaw"],
        J_NAME_TO_ID["left_arm_elbow_pitch"],
        J_NAME_TO_ID["right_arm_shoulder_yaw"],
        J_NAME_TO_ID["right_arm_elbow_pitch"],
    ]
    arm_type_zero_ids = [J_NAME_TO_ID["left_arm_elbow_roll"], J_NAME_TO_ID["right_arm_elbow_roll"]]

    # Configure actuators for each type
    # Legs
    # configure_actuators(kos, leg_type_four_ids, kp=120, kd=10, max_torque=20)
    # configure_actuators(kos, leg_type_three_ids, kp=60, kd=5, max_torque=15)
    # configure_actuators(kos, leg_type_two_ids, kp=34, kd=5, max_torque=10)

    # Arms
    configure_actuators(kos, arm_type_three_ids, kp=80, kd=5, max_torque=20)
    configure_actuators(kos, arm_type_two_ids, kp=45, kd=3, max_torque=15)
    configure_actuators(kos, arm_type_zero_ids, kp=20, kd=1, max_torque=5)

    actuator_ids = arm_type_three_ids + arm_type_two_ids + arm_type_zero_ids

    # Get the starting position of each actuator
    JOINT_START_POS = {}
    for actuator_id in actuator_ids:
        try:
            state = kos.actuator.get_actuators_state(actuator_ids=[actuator_id])
            JOINT_START_POS[actuator_id] = state[0].position
            print(f"Actuator {actuator_id:<2} - {J_ID_TO_NAME[actuator_id]:<25} position is {state[0].position:>7.3f}")
        except Exception as e:
            print(f"Failed to get state for actuator {actuator_id} - {J_ID_TO_NAME[actuator_id]}: {e}")


def on_press(key):
    global key_pressed
    key_pressed = key


def on_release(key):
    global key_pressed
    key_pressed = None
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def parse_movement_command(cmd, default_step=STEP_SIZE):
    """
    Parse movement command of format 'u [number]' or 'd [number]'.

    Args:
        cmd: Input command string
        default_step: Default step size if no number provided

    Returns:
        tuple: (direction_multiplier, step_size)
        direction_multiplier: 1 for up, -1 for down, 0 for invalid/quit
        step_size: Number of degrees to move
    """
    cmd = cmd.lower().strip()

    if cmd == "q":
        return (0, 0)

    if not (cmd.startswith("u") or cmd.startswith("d")):
        return (0, 0)

    direction = 1 if cmd.startswith("u") else -1

    try:
        # Try to get the number after the command, use default if not provided
        step = default_step
        if len(cmd) > 1:
            step = float(cmd[1:])
    except ValueError:
        print(f"Invalid number provided")
        return (0, 0)

    return (direction, step)


def control_actuator_with_keyboard(kos, actuator_id):
    """
    Control an actuator using keyboard inputs with safety limits.

    Args:
        kos: KOS instance
        actuator_id: ID of the motor to control
    """
    try:
        print(f"\nControlling actuator {actuator_id} - {J_ID_TO_NAME[actuator_id]}")
        print(f"Step size: {STEP_SIZE} degrees")
        print("Enter commands:")
        print("'u' for up")
        print("'d' for down")
        print("'q' to quit")

        while True:
            # Get current position
            state = kos.actuator.get_actuators_state(actuator_ids=[actuator_id])
            current_position = state[0].position

            desired_position = current_position

            # Get and parse command from user
            cmd = input("> ")
            direction, step = parse_movement_command(cmd)

            if direction == 0:  # Invalid command or quit
                if cmd == "q":
                    print("Exiting...")
                    break
                continue

            # Calculate desired position
            desired_position = current_position + (direction * step)
            print(f"Moving {'up' if direction > 0 else 'down'}: {current_position:.2f} -> {desired_position:.2f}")

            # Safety check: limit movement to ±5 degrees from current position
            clipped_position = max(current_position - MAX_STEP, min(current_position + MAX_STEP, desired_position))

            # Only send command if position changed
            print("Before command:")
            print(f"Current position is: {current_position:.3f}")
            print(f"Sending command: {clipped_position:.3f}")
            print(f"Difference is: {clipped_position - current_position:.3f}")

            if clipped_position != current_position:
                command = {"actuator_id": actuator_id, "position": clipped_position}
                kos.actuator.command_actuators(commands=[command])

            time.sleep(0.1)

            # Get position after command
            state = kos.actuator.get_actuators_state(actuator_ids=[actuator_id])
            actual_position = state[0].position

            print(f"\nAfter command:")
            print(f"Actual position reached: {actual_position:.3f}")
            print(f"Command position was: {clipped_position:.3f}")
            print(f"Position error: {actual_position - clipped_position:.3f}")

    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        # Cleanup: disable torque
        print("Disabling torque!")
        kos.actuator.configure_actuator(actuator_id=actuator_id, torque_enabled=False)


def main():
    # Initialize KOS and setup
    kos = pykos.KOS()
    set_up(kos)

    # You can change this to control different actuators
    actuator_id = J_NAME_TO_ID["right_arm_shoulder_roll"]
    control_actuator_with_keyboard(kos, actuator_id)


if __name__ == "__main__":
    main()
