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
        1: "04",
        2: "01",
    }
}

DELTA_POSITION = 0.5  # radians / second
DELTA_POSITION_BIG_MUL = 6.0

DELTA_VELOCITY = 0.5  # radians / second ^ 2
DELTA_VELOCITY_BIG_MUL = 6.0

DELTA_KP = 0.1
DELTA_KP_BIG = 10.0

DELTA_KD = 0.01
DELTA_KD_BIG = 1.0

# Update color constants
BOLD_GREEN = "\033[1;32m"  # Bold Green
GREY = "\033[90m"  # Grey
RESET = "\033[0m"


@dataclass
class SharedState:
    joystick_x: float = 0.0
    joystick_y: float = 0.0
    direction_pad_y: int = 0
    direction_pad_x: int = 0
    right_toggle: bool = False
    left_toggle: bool = False
    active_motor_index: int = 0
    red_button_pressed: bool = False
    green_button_pressed: bool = False
    yellow_button_pressed: bool = False
    blue_button_pressed: bool = False


def gamepad_thread(shared_state: SharedState) -> None:
    while True:
        events = get_gamepad()
        for event in events:
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

            # Direction pad up/down.
            if event.code == "ABS_HAT0Y":
                shared_state.direction_pad_y = event.state

            # Direction pad left/right.
            if event.code == "ABS_HAT0X":
                shared_state.direction_pad_x = event.state

            # Red button.
            if event.code == "BTN_EAST":
                shared_state.red_button_pressed = event.state == 1

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
                target_update_rate=10000.0,
            ),
            port_name,
            motor_infos,
        )
        for port_name, motor_infos in PORT_NAMES.items()
    ]

    shared_state = SharedState()

    gamepad_thread_obj = threading.Thread(target=gamepad_thread, args=(shared_state,), daemon=True)
    gamepad_thread_obj.start()

    all_motors: list[tuple[RobstrideMotorsSupervisor, str, int]] = []
    for motor, port_name, info in motors:
        for motor_id in info.keys():
            all_motors.append((motor, port_name, motor_id))

    active_motor_index = 0
    num_motors = len(all_motors)

    log_last_time = last_time = time.time()

    try:
        while True:
            time.sleep(0.025)

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # Move to the next/previous motor
            if shared_state.left_toggle or shared_state.right_toggle:
                shared_state.left_toggle = shared_state.right_toggle = False
                active_motor_index = (active_motor_index + (1 if shared_state.left_toggle else -1)) % num_motors
                _, port_name, motor_id = all_motors[active_motor_index]
                print(
                    f"Active motor: {BOLD_GREEN}{active_motor_index + 1}{RESET}/{BOLD_GREEN}{num_motors}{RESET} - {port_name} ID {BOLD_GREEN}{motor_id}{RESET}"
                )

            active_motor, port_name, motor_id = all_motors[active_motor_index]

            if current_time - log_last_time > 1.0:
                log_last_time = current_time
                failed_commands = active_motor.get_failed_commands()
                total_commands = active_motor.get_total_commands()
                update_rate = active_motor.get_actual_update_rate()
                print(
                    f"{GREY}Failed commands: {failed_commands}/{total_commands} - Update rate: {update_rate:.2f} Hz{RESET}"
                )

            # Zero the active motor.
            if shared_state.red_button_pressed:
                shared_state.red_button_pressed = False
                active_motor.add_motor_to_zero(motor_id)
                print(f"Zeroing active motor {BOLD_GREEN}{motor_id}{RESET} on {port_name}")

            # Change the active motor's target position.
            if abs(shared_state.joystick_x) > 0.15:  # Add a small deadzone
                mul = shared_state.joystick_x * DELTA_POSITION * dt
                if shared_state.blue_button_pressed:
                    mul *= DELTA_POSITION_BIG_MUL
                position = active_motor.get_position(motor_id) + mul
                active_motor.set_position(motor_id, position)
                print(f"Active motor target position: {BOLD_GREEN}{position:.2f}{RESET}")

            # Change the active motor's target velocity.
            if abs(shared_state.joystick_y) > 0.15:
                mul = shared_state.joystick_y * DELTA_VELOCITY * dt
                if shared_state.blue_button_pressed:
                    mul *= DELTA_VELOCITY_BIG_MUL
                velocity = active_motor.get_velocity(motor_id) + mul
                active_motor.set_velocity(motor_id, velocity)
                print(f"Active motor target velocity: {BOLD_GREEN}{velocity:.2f}{RESET}")
            else:
                active_motor.set_velocity(motor_id, 0.0)

            # Change the active motor's KP gain.
            if shared_state.direction_pad_y != 0:
                if shared_state.blue_button_pressed:
                    kp = active_motor.get_kp(motor_id)
                    kp = (round(kp / DELTA_KP_BIG) - shared_state.direction_pad_y) * DELTA_KP_BIG
                    shared_state.direction_pad_y = 0
                    active_motor.set_kp(motor_id, kp)
                    print(f"kp: {BOLD_GREEN}{kp:.2f}{RESET}")
                else:
                    kp = active_motor.get_kp(motor_id)
                    kp += -shared_state.direction_pad_y * DELTA_KP
                    active_motor.set_kp(motor_id, kp)
                    print(f"kp: {BOLD_GREEN}{kp:.2f}{RESET}")

            # Change the active motor's KD gain.
            if shared_state.direction_pad_x != 0:
                if shared_state.blue_button_pressed:
                    kd = active_motor.get_kd(motor_id)
                    kd += shared_state.direction_pad_x * DELTA_KD_BIG
                    shared_state.direction_pad_x = 0
                    active_motor.set_kd(motor_id, kd)
                    print(f"kd: {BOLD_GREEN}{kd:.2f}{RESET}")
                else:
                    kd = active_motor.get_kd(motor_id)
                    kd += shared_state.direction_pad_x * DELTA_KD
                    active_motor.set_kd(motor_id, kd)
                    print(f"kd: {BOLD_GREEN}{kd:.2f}{RESET}")

    except KeyboardInterrupt:
        for motor, _, _ in motors:
            motor.stop()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
