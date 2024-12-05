import time
from actuator import RobstrideMotorFeedback, RobstrideMotorsSupervisor

motor_configs = {
    # Left arm
    "/dev/ttyCH341USB0" : {
        1: "03",
        2: "03",
        3: "02",
        4: "02",
        5: "02",
        6: "00",
    },
    # Right arm
    "/dev/ttyCH341USB1": {
        1: "03",
        2: "03",
        3: "02",
        4: "02",
        5: "02",
        6: "00",
    },
    # Left leg
    "/dev/ttyCH341USB2": {
        1: "04",
        2: "03",
        3: "03",
        4: "03",
        5: "02",
    },
    # Right leg
    "/dev/ttyCH341USB3": {
        1: "04",
        2: "03",
        3: "03",
        4: "03",
        5: "02",
    },
}

def main() -> None:
    motors = {}
    for port_name, motor_config in motor_configs.items():
        motors[port_name] = RobstrideMotorsSupervisor(port_name, motor_config)

    time.sleep(1)

    positions = {motor_supervisor : {motor_id : motor_supervisor.get_latest_feedback()[motor_id].position
                                      for motor_id in motor_configs[port]} 
                 for port, motor_supervisor in motors.items()}
    print(positions)
    time.sleep(5)

    for motor_supervisor, motor_positions in positions.items():
        for motor_id, motor_position in motor_positions.items():
            motor_supervisor.set_position(motor_id, motor_position)
    
    time.sleep(1)

    for motor_supervisor, motor_positions in positions.items():
        for motor_id in motor_positions.keys():
            motor_supervisor.set_kp(motor_id, 30)
            motor_supervisor.set_kd(motor_id, 1)

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
