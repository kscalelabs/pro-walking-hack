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

STILL_MOVING_TORQUE = 0.1
TORQUE_LIMIT = 1.0


@dataclass
class SharedState:
    left_is_down: bool = False
    right_is_down: bool = False
    up_is_down: bool = False
    down_is_down: bool = False
    right_toggle: bool = False
    left_toggle: bool = False
    zero_position: bool = False


def gamepad_thread(shared_state: SharedState) -> None:
    while True:
        events = get_gamepad()
        for event in events:
            # Red button.
            if event.code == "BTN_EAST":
                if event.state == 1:
                    shared_state.zero_position = True
                else:
                    shared_state.zero_position = False

            # Right toggle
            if event.code == "BTN_TR":
                if event.state == 1:
                    shared_state.right_toggle = True
                else:
                    shared_state.right_toggle = False

            # Left toggle
            if event.code == "BTN_TL":
                if event.state == 1:
                    shared_state.left_toggle = True
                else:
                    shared_state.left_toggle = False

            # Left-right on pad.
            if event.code == "ABS_X":
                if event.state > 1000:
                    shared_state.left_is_down = True
                elif event.state < -1000:
                    shared_state.right_is_down = True
                else:
                    shared_state.left_is_down = False
                    shared_state.right_is_down = False

            # Up-down on pad.
            if event.code == "ABS_Y":
                if event.state < -1000:
                    shared_state.up_is_down = True
                elif event.state > 1000:
                    shared_state.down_is_down = True
                else:
                    shared_state.up_is_down = False
                    shared_state.down_is_down = False


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

    for motor, _, info in motors:
        for motor_id in info.keys():
            motor.add_motor_to_zero(motor_id)

    shared_state = SharedState()

    gamepad_thread_obj = threading.Thread(target=gamepad_thread, args=(shared_state,), daemon=True)
    gamepad_thread_obj.start()

    target_position = 0.0
    kp = 10.0
    kd = 1.0

    try:
        while True:
            if shared_state.left_is_down:
                target_position += 0.1
            elif shared_state.right_is_down:
                target_position -= 0.1

            if shared_state.up_is_down or shared_state.down_is_down:
                if shared_state.up_is_down:
                    if shared_state.right_toggle:
                        kd += 0.1
                    else:
                        kp += 0.1
                elif shared_state.down_is_down:
                    if shared_state.right_toggle:
                        kd -= 0.1
                    else:
                        kp -= 0.1
                print(f"kp: {kp:.2f}, kd: {kd:.2f}")

                for motor, _, info in motors:
                    for motor_id in info.keys():
                        motor.set_kp_kd(motor_id, kp, kd)

            for motor, _, info in motors:
                for motor_id in info.keys():
                    motor.set_target_position(motor_id, target_position)

            if shared_state.zero_position:
                target_position = 0.0
                for motor, _, info in motors:
                    for motor_id in info.keys():
                        motor.add_motor_to_zero(motor_id)

            time.sleep(0.025)  # Add a small delay to prevent excessive CPU usage

    except KeyboardInterrupt:
        for motor, _, _ in motors:
            motor.stop()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
