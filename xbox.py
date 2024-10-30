"""Freezes the motors in place using PD control."""

import threading
import time
from dataclasses import dataclass

import pygame
from actuator import RobstrideMotorsSupervisor

# Initialize pygame
pygame.init()
pygame.joystick.init()

leg_config = {
    1 : "04",
    2 : "03",
    3 : "03",
    4 : "04",
    5 : "02",
}

arm_config = {
    1 : "03",
    2 : "03",
    3 : "02",
    4 : "02",
    5 : "02",
    6 : "00",
}

PORT_NAMES = {
    # Right leg.
    # "/dev/ttyCH341USB0": leg_config,
    "/dev/ttyCH341USB1" : arm_config,
    # # Right leg.
    # "/dev/ttyUSB1": {
    #     1: "04",
    #     2: "03",
    #     3: "03",
    #     4: "03",
    #     # 5: "01",
    # },
}

DELTA_POSITION = 0.5  # radians / second
DELTA_POSITION_BIG_MUL = 6.0

DELTA_VELOCITY = 0.5  # radians / second ^ 2
DELTA_VELOCITY_BIG_MUL = 6.0

DELTA_KP = 0.5
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
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        pygame.event.pump()

        # Right toggle
        shared_state.right_toggle = joystick.get_button(5)  # RB

        # Left toggle
        shared_state.left_toggle = joystick.get_button(4)  # LB

        # Joystick X-axis
        shared_state.joystick_x = joystick.get_axis(0)  # Left stick X

        # Joystick Y-axis
        shared_state.joystick_y = -joystick.get_axis(1)  # Left stick Y

        # Direction pad up/down.
        shared_state.direction_pad_y = joystick.get_hat(0)[1]

        # Direction pad left/right.
        shared_state.direction_pad_x = joystick.get_hat(0)[0]

        # Red button.
        shared_state.red_button_pressed = joystick.get_button(1)  # B

        # Green button.
        shared_state.green_button_pressed = joystick.get_button(0)  # A

        # Yellow button.
        shared_state.yellow_button_pressed = joystick.get_button(3)  # Y

        # Blue button.
        shared_state.blue_button_pressed = joystick.get_button(2)  # X

        time.sleep(0.05)  # Add a small delay to reduce CPU usage


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
                active_motor_index = (active_motor_index + (1 if shared_state.right_toggle else -1)) % num_motors
                shared_state.left_toggle = shared_state.right_toggle = False
                _, port_name, motor_id = all_motors[active_motor_index]
                print(
                    f"Active motor: {BOLD_GREEN}{active_motor_index + 1}{RESET}/{BOLD_GREEN}{num_motors}{RESET} - "
                    f"{port_name} ID {BOLD_GREEN}{motor_id}{RESET}"
                )

            active_motor, port_name, motor_id = all_motors[active_motor_index]

            if not active_motor.is_running():
                raise Exception("Motor is not running")

            if current_time - log_last_time > 1.0:
                log_last_time = current_time
                failed_commands = active_motor.failed_commands_for(motor_id)
                total_commands = active_motor.total_commands
                update_rate = active_motor.actual_update_rate
                print(
                    f"{GREY}Failed commands: {failed_commands}/{total_commands} - "
                    f"Update rate: {update_rate:.2f} Hz{RESET}"
                )

            if shared_state.red_button_pressed:
                shared_state.red_button_pressed = False

                # Zero the active motor.
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
                    kd = (round(kd / DELTA_KD_BIG) + shared_state.direction_pad_x) * DELTA_KD_BIG
                    shared_state.direction_pad_x = 0
                    kd = active_motor.set_kd(motor_id, kd)
                    print(f"kd: {BOLD_GREEN}{kd:.2f}{RESET}")
                else:
                    kd = active_motor.get_kd(motor_id)
                    kd += shared_state.direction_pad_x * DELTA_KD
                    kd = active_motor.set_kd(motor_id, kd)
                    print(f"kd: {BOLD_GREEN}{kd:.2f}{RESET}")

    except KeyboardInterrupt:
        for motor, _, _ in motors:
            motor.stop()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
