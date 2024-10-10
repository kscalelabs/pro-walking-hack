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


def main() -> None:
    motors = [
        (
            RobstrideMotors(
                port_name=port_name,
                motor_infos=motor_infos,
            ),
            motor_infos,
        )
        for port_name, motor_infos in PORT_NAMES.items()
    ]

    def get_torque(target_position: float, feedback: RobstrideMotorFeedback) -> float:
        cur_position = feedback.position
        cur_velocity = feedback.velocity
        target_velocity = 0.0

        # Position plus velocity.
        kp = 5.0
        kd = 0.1

        # Just position.
        # kp = 0.05
        # kd = 0.0

        torque = kp * (target_position - cur_position) + kd * (target_velocity - cur_velocity)
        return torque

    # Initialize the motors.
    for motor, info in motors:
        motor.send_reset()
        if rs_01 := [i for i, kind in info.items() if kind == "01"]:
            motor.send_set_zero(rs_01)  # Zero the single-encoder motors.
        motor.send_start()

    try:
        start_time = time.time()
        while True:
            target_position = math.sin(time.time() - start_time) * math.pi

            for motor, info in motors:
                feedback = motor.get_latest_feedback()
                torques = {i: get_torque(target_position, feedback) for i, feedback in feedback.items()}
                motor.send_torque_controls(torques)

    except KeyboardInterrupt:
        print("Resetting motors")
        for motor, _ in motors:
            motor.send_torque_controls({i: 0.0 for i in info})

        time.sleep(0.1)
        for motor, _ in motors:
            motor.send_reset()
        time.sleep(0.1)


if __name__ == "__main__":
    # python freeze.py
    main()
