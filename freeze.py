"""Freezes the motors in place using PD control."""

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

MAX_TORQUE = 10.0


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

    def get_torque(id: int, info: dict[int, str], feedback: RobstrideMotorFeedback) -> float:
        cur_position = feedback.position
        cur_velocity = feedback.velocity
        target_position = 0.0
        target_velocity = 0.0
        kp = 3.0
        kd = 0.1
        torque = kp * (target_position - cur_position) + kd * (target_velocity - cur_velocity)
        return min(max(torque, -MAX_TORQUE), MAX_TORQUE)

    # Initialize the motors.
    for motor, _ in motors:
        motor.send_reset()
        motor.send_set_zero()
        motor.send_start()

    try:
        while True:
            for motor, info in motors:
                feedback = motor.get_latest_feedback()
                torques = {i: get_torque(i, info, feedback) for i, feedback in feedback.items()}
                motor.send_torque_controls(torques)

    except KeyboardInterrupt:
        print("Resetting motors")
        for motor, _ in motors:
            motor.send_torque_controls({i: 0.0 for i in info})
            motor.send_reset()


if __name__ == "__main__":
    # python freeze.py
    main()
