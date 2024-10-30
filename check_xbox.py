"""Simulates motor control actions for gamepad testing."""

import threading
import time
from dataclasses import dataclass

import pygame

# Initialize pygame
pygame.init()
pygame.joystick.init()

PORT_NAMES = {
    # Left leg.
    "/dev/ttyUSB0": {
        1: "04",
        2: "03",
        3: "03",
        4: "03",
        5: "01",
    },
    # Right leg.
    "/dev/ttyUSB1": {
        1: "04",
        2: "03",
        3: "03",
        4: "03",
        # 5: "01",
    },
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
    shared_state = SharedState()

    gamepad_thread_obj = threading.Thread(target=gamepad_thread, args=(shared_state,), daemon=True)
    gamepad_thread_obj.start()

    active_motor_index = 0
    num_motors = sum(len(info) for info in PORT_NAMES.values())

    try:
        while True:
            time.sleep(0.025)

            # Move to the next/previous motor
            if shared_state.left_toggle or shared_state.right_toggle:
                active_motor_index = (active_motor_index + (1 if shared_state.right_toggle else -1)) % num_motors
                shared_state.left_toggle = shared_state.right_toggle = False
                print(
                    f"Active motor: {BOLD_GREEN}{active_motor_index + 1}{RESET}/{BOLD_GREEN}{num_motors}{RESET}"
                )

            if shared_state.red_button_pressed:
                shared_state.red_button_pressed = False
                print(f"Zeroing active motor {BOLD_GREEN}{active_motor_index + 1}{RESET}")

            # Simulate changing the active motor's target position.
            if abs(shared_state.joystick_x) > 0.15:  # Add a small deadzone
                mul = shared_state.joystick_x * DELTA_POSITION
                if shared_state.blue_button_pressed:
                    mul *= DELTA_POSITION_BIG_MUL
                print(f"Active motor target position change: {BOLD_GREEN}{mul:.2f}{RESET}")

            # Simulate changing the active motor's target velocity.
            if abs(shared_state.joystick_y) > 0.15:
                mul = shared_state.joystick_y * DELTA_VELOCITY
                if shared_state.blue_button_pressed:
                    mul *= DELTA_VELOCITY_BIG_MUL
                print(f"Active motor target velocity change: {BOLD_GREEN}{mul:.2f}{RESET}")

            # Simulate changing the active motor's KP gain.
            if shared_state.direction_pad_y != 0:
                if shared_state.blue_button_pressed:
                    kp_change = -shared_state.direction_pad_y * DELTA_KP_BIG
                else:
                    kp_change = -shared_state.direction_pad_y * DELTA_KP
                shared_state.direction_pad_y = 0
                print(f"KP change: {BOLD_GREEN}{kp_change:.2f}{RESET}")

            # Simulate changing the active motor's KD gain.
            if shared_state.direction_pad_x != 0:
                if shared_state.blue_button_pressed:
                    kd_change = shared_state.direction_pad_x * DELTA_KD_BIG
                else:
                    kd_change = shared_state.direction_pad_x * DELTA_KD
                shared_state.direction_pad_x = 0
                print(f"KD change: {BOLD_GREEN}{kd_change:.2f}{RESET}")

    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
