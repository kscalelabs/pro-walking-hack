"""Freezes the motors in place using PD control."""

import threading
import time
from dataclasses import dataclass

from actuator import RobstrideMotorsSupervisor
from inputs import get_gamepad

# PORT_NAMES = {
#     # Left leg.
#     "/dev/ttyUSB0": {
#         1: "04",
#         2: "03",
#         3: "03",
#         4: "03",
#         5: "01",
#     },
#     # Right leg.
#     "/dev/ttyUSB1": {
#         1: "04",
#         2: "03",
#         3: "03",
#         4: "03",
#         5: "01",
#     },
# }

PORT_NAMES = {
    # Test setup.
    "/dev/ttyUSB0": {
        2: "01",
    }
}

DELTA_POSITION = 0.1
DELTA_KP = 0.4
DELTA_KD = 0.02


@dataclass
class SharedState:
    joystick_x: float = 0.0
    joystick_y: float = 0.0
    right_toggle: bool = False
    left_toggle: bool = False
    zero_position: bool = False
    active_motor_index: int = 0
    green_button_pressed: bool = False
    yellow_button_pressed: bool = False
    blue_button_pressed: bool = False


def gamepad_thread(shared_state: SharedState) -> None:
    while True:
        events = get_gamepad()
        for event in events:
            # Red button.
            if event.code == "BTN_EAST":
                shared_state.zero_position = event.state == 1

            # Right toggle
            if event.code == "BTN_TR":
                shared_state.right_toggle = event.state == 1

            # Left toggle
            if event.code == "BTN_TL":
                shared_state.left_toggle = event.state == 1

            # Joystick X-axis
            if event.code == "ABS_X":
                shared_state.joystick_x = event.state / 32768.0  # Normalize to [-1, 1]

            # Joystick Y-axis
            if event.code == "ABS_Y":
                shared_state.joystick_y = -event.state / 32768.0  # Normalize to [-1, 1]

            # Green button.
            if event.code == "BTN_SOUTH":
                shared_state.green_button_pressed = event.state == 1

            # Yellow button.
            if event.code == "BTN_WEST":
                shared_state.yellow_button_pressed = event.state == 1

            # Blue button.
            if event.code == "BTN_NORTH":
                shared_state.blue_button_pressed = event.state == 1


def main() -> None:
    motors = [
        (
            RobstrideMotorsSupervisor(
                port_name=port_name,
                motor_infos=motor_infos,
            ),
            port_name,
            motor_infos,
        )
        for port_name, motor_infos in PORT_NAMES.items()
    ]

    # Zero all the 01 motors on startup.
    for motor, _, info in motors:
        for motor_id, _ in info.items():
            motor.add_motor_to_zero(motor_id)

    shared_state = SharedState()

    gamepad_thread_obj = threading.Thread(target=gamepad_thread, args=(shared_state,), daemon=True)
    gamepad_thread_obj.start()

    target_position = 0.0
    kp = 10.0
    kd = 1.0

    all_motors: list[tuple[RobstrideMotorsSupervisor, str, int]] = []
    for motor, port_name, info in motors:
        for motor_id in info.keys():
            all_motors.append((motor, port_name, motor_id))

    active_motor_index = 0
    num_motors = len(all_motors)

    try:
        while True:
            if shared_state.green_button_pressed:
                shared_state.green_button_pressed = False
                active_motor_index = (active_motor_index + 1) % num_motors
                _, port_name, active_motor_id = all_motors[active_motor_index]
                print(f"Active motor: {active_motor_index + 1}/{num_motors} - {port_name} ID {active_motor_id}")

            if shared_state.yellow_button_pressed:
                shared_state.yellow_button_pressed = False
                active_motor_index = (active_motor_index - 1) % num_motors
                _, port_name, active_motor_id = all_motors[active_motor_index]
                print(f"Active motor: {active_motor_index + 1}/{num_motors} - {port_name} ID {active_motor_id}")

            # Zero all motors.
            if shared_state.blue_button_pressed:
                shared_state.blue_button_pressed = False
                for motor, port_name, motor_id in all_motors:
                    motor.add_motor_to_zero(motor_id)
                    print(f"Zeroing motor {motor_id} on {port_name}")

            active_motor, _, active_motor_id = all_motors[active_motor_index]

            if abs(shared_state.joystick_x) > 0.05:  # Add a small deadzone
                target_position += shared_state.joystick_x * DELTA_POSITION
                print(f"target_position: {target_position:.2f}")
                active_motor.set_target_position(active_motor_id, target_position)

            if abs(shared_state.joystick_y) > 0.25:  # Add a small deadzone
                if shared_state.right_toggle:
                    kd += (shared_state.joystick_y - 0.25) * DELTA_KD
                else:
                    kp += (shared_state.joystick_y - 0.25) * DELTA_KP
                print(f"kp: {kp:.2f}, kd: {kd:.2f}")
                active_motor.set_kp_kd(active_motor_id, kp, kd)

            if shared_state.zero_position:
                target_position = 0.0
                active_motor.set_target_position(active_motor_id, target_position)
                active_motor.add_motor_to_zero(active_motor_id)
                print("zeroing active motor")
                shared_state.zero_position = False

            time.sleep(0.025)  # Add a small delay to prevent excessive CPU usage

    except KeyboardInterrupt:
        for motor, _, _ in motors:
            motor.stop()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
