import time
from actuator import RobstrideMotorFeedback, RobstrideMotorsSupervisor

motor_configs = {
    # Left arm
    # "/dev/ttyCH341USB0" : {
    #     11: "03",
    #     12: "03",
    #     13: "02",
    #     14: "02",
    #     15: "02",
    #     16: "00",
    # },
    # Right arm
    "/dev/ttyCH341USB1": {
        1: "03",
        2: "03",
        3: "02",
        4: "02",
        5: "02",
        6: "00",
    },
    # # Left leg
    # "/dev/ttyCH341USB2": {
    #     21: "04",
    #     22: "03",
    #     23: "03",
    #     24: "03",
    #     25: "02",
    # },
    # Right leg
    "/dev/ttyCH341USB3": {
        31: "04",
        32: "03",
        33: "03",
        34: "03",
        35: "02",
    },
}

def main() -> None:
    motors = {}
    for port_name, motor_config in motor_configs.items():
        motors[port_name] = RobstrideMotorsSupervisor(port_name, motor_config)

    time.sleep(1)

    while True:
        time.sleep(0.2)
        positions = {motor_supervisor : {motor_id : motor_supervisor.get_latest_feedback()[motor_id].position
                                        for motor_id in motor_configs[port]} 
                    for port, motor_supervisor in motors.items()}

        print(f"Left arm: {positions['/dev/ttyCH341USB0']}")
        print(f"Right arm: {positions['/dev/ttyCH341USB1']}")
        print(f"Left leg: {positions['/dev/ttyCH341USB2']}")
        print(f"Right leg: {positions['/dev/ttyCH341USB3']}")


if __name__ == "__main__":
    main()
