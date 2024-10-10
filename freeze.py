"""Freezes the motors in place using PD control."""

import math
import time

from actuator import RobstrideMotorFeedback, RobstrideMotors

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


def main() -> None:
    motors = [
        (
            RobstrideMotors(
                port_name=port_name,
                motor_infos=motor_infos,
            ),
            port_name,
            motor_infos,
        )
        for port_name, motor_infos in PORT_NAMES.items()
    ]

    def get_torque(target_position: float, feedback: RobstrideMotorFeedback) -> float:
        cur_position = feedback.position
        cur_velocity = feedback.velocity
        target_velocity = 0.0

        # Position plus velocity.
        kp = 1.0
        kd = 0.3

        # Just position.
        # kp = 0.05
        # kd = 0.0

        torque = kp * (target_position - cur_position) + kd * (target_velocity - cur_velocity)
        return min(max(torque, -TORQUE_LIMIT), TORQUE_LIMIT)

    # Initialize the motors.
    for motor, _, _ in motors:
        motor.send_reset()
        motor.send_start()

    def get_motor_positions() -> dict[str, dict[int, float]]:
        motor_positions: dict[str, dict[int, float]] = {}
        for motor, port_name, info in motors:
            feedback = motor.get_latest_feedback()
            if rs_01 := [i for i, kind in info.items() if kind == "01"]:
                motor_positions[port_name] = {i: feedback[i].position for i in rs_01}
        return motor_positions

    def move_until_stopped(k_dp: float) -> dict[str, dict[int, float]]:
        start_positions = get_motor_positions()
        still_moving = True
        start_time = time.time()
        while still_moving:
            elapsed_time = time.time() - start_time
            dp = elapsed_time * k_dp
            still_moving = elapsed_time < 0.5
            for motor, port_name, info in motors:
                if port_name not in start_positions:
                    continue
                if rs_01 := [i for i, kind in info.items() if kind == "01"]:
                    feedback = motor.get_latest_feedback()
                    torques = {i: get_torque(start_positions[port_name][i] + dp, feedback[i]) for i in rs_01}
                    motor.send_torque_controls(torques)
                    still_moving = still_moving or any(abs(torque) < STILL_MOVING_TORQUE for torque in torques.values())
        return get_motor_positions()

    def zero_motors() -> None:
        # Slowly move backwards from the starting position until all motors have stopped moving.
        start_positions = move_until_stopped(k_dp=math.pi)
        end_positions = move_until_stopped(k_dp=-math.pi)

        # Moves to the middle of the range.
        mid_positions = {
            port_name: {
                i: (start_positions[port_name][i] + end_positions[port_name][i]) / 2 for i in start_positions[port_name]
            }
            for port_name in start_positions
        }

        still_moving = True
        while still_moving:
            still_moving = False
            for motor, port_name, info in motors:
                if rs_01 := [i for i, kind in info.items() if kind == "01"]:
                    feedback = motor.get_latest_feedback()
                    torques = {i: get_torque(mid_positions[port_name][i], feedback[i]) for i in rs_01}
                    motor.send_torque_controls(torques)
                    still_moving = still_moving or any(
                        abs(feedback[i].position - mid_positions[port_name][i]) > 0.1 for i in rs_01
                    )

        for motor, _, info in motors:
            if rs_01 := [i for i, kind in info.items() if kind == "01"]:
                motor.send_torque_controls({i: 0.0 for i in rs_01})
                motor.send_set_zero(rs_01)

    zero_motors()

    try:
        # start_time = time.time()
        while True:
            # target_position = math.sin(time.time() - start_time) * math.pi
            target_position = 0.0

            for motor, _, info in motors:
                feedback = motor.get_latest_feedback()
                torques = {i: get_torque(target_position, feedback) for i, feedback in feedback.items()}
                # torques = {i: 0.0 for i in info}
                motor.send_torque_controls(torques)

    except KeyboardInterrupt:
        for motor, _, _ in motors:
            motor.send_torque_controls({i: 0.0 for i in info})
        time.sleep(0.1)

        for motor, _, _ in motors:
            motor.send_reset()
        time.sleep(0.1)


if __name__ == "__main__":
    # python freeze.py
    main()
