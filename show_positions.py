"""Repeatedly shows the positions of the motors."""

import time

from actuator import RobstrideMotorsSupervisor

PORT_NAMES = {
    # Test setup.
    "/dev/ttyUSB0": {
        2: "01",
    }
}


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

    for motor, _, _ in motors:
        motor.send_reset()
        motor.send_start()

    try:
        print("Press Enter to show positions. Press Ctrl+C to exit.", end="")
        while True:
            input("")  # Wait for user to press Enter
            positions: dict[str, dict[int, float]] = {}
            for motor, port_name, info in motors:
                torques = {i: 0.0 for i in info.keys()}
                feedback = motor.send_torque_controls(torques)
                positions[port_name] = {i: feedback[i].position for i in info.keys()}
            print(positions, end="")

    except KeyboardInterrupt:
        for motor, _, _ in motors:
            motor.send_reset()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
