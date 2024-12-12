import pykos
import argparse
import numpy as np
import time

kos = pykos.KOS()

ALL_IDS = [
    21, 22, 23, 24, 25, 26, # arm
    31, 32, 33, 34, 35,
    41, 42, 43, 44, 45]

def set_up(kos):
    arm_ids = [21, 22, 23, 24, 25, 26]
    left_leg_ids = [31, 32, 33, 34, 35]
    right_leg_ids = [41, 42, 43, 44, 45]

    type_four_ids = [limb[id] for limb in [left_leg_ids, right_leg_ids] for id in [0, 3]]
    type_three_ids = [limb[id] for limb in [left_leg_ids, right_leg_ids] for id in [1, 2]]
    type_two_ids = [limb[id] for limb in [left_leg_ids, right_leg_ids] for id in [4]]

    # Configure all motors
    for id in type_four_ids:
        print(f"Configuring actuator {id}")
        kos.actuator.configure_actuator(actuator_id=id, kp=120, kd=10, max_torque=20, torque_enabled=True)

    for id in type_three_ids:
        print(f"Configuring actuator {id}")
        kos.actuator.configure_actuator(actuator_id=id, kp=60, kd=5, max_torque=15, torque_enabled=True)

    for id in type_two_ids:
        print(f"Configuring actuator {id}")
        kos.actuator.configure_actuator(actuator_id=id, kp=34, kd=5, max_torque=10, torque_enabled=True)

    for id in arm_ids:
        print(f"Configuring actuator {id}")
        kos.actuator.configure_actuator(actuator_id=id, kp=5, kd=5, max_torque=5, torque_enabled=True)

    state = kos.actuator.get_actuators_state(actuator_ids=ALL_IDS)
    for s in state:
        print(f"Actuator {s.actuator_id} is {s.position:.3f}")

def main(motor_id=42, kp=1.0, kd=0.1, period=1.0, amplitude=np.pi/4, max_torque=40.0):
    try:
        try:
            kos.process_manager.start_kclip("sine_wave_motor_{motor_id}_kp_{kp}_kd_{kd}_period_{period}_amplitude_{amplitude}_max_torque_{max_torque}")
            set_up(kos)
        except Exception as e:
            print(e)
        # Configure the actuator first
        print(f"Configuring actuator {motor_id}")
        kos.actuator.configure_actuator(
            actuator_id=motor_id, 
            kp=kp, 
            kd=kd, 
            max_torque=max_torque,
            torque_enabled=True
        )

        start_time = time.time()
        while True:
            # Calculate sine wave position: amplitude * sin(2π * t/T)
            state = kos.actuator.get_actuators_state(actuator_ids=[motor_id])
            print(f"Actuator {motor_id} is {state[0].position:.3f}")
            t = time.time() - start_time
            position = amplitude * np.sin(2 * np.pi * t / period) * 180 / np.pi - 0.441 * 180 / np.pi
            
            # Command the actuator
            command = {'actuator_id': motor_id, 'position': position}
            kos.actuator.command_actuators(commands=[command])
            
            time.sleep(0.01)  # 100Hz control loop

    except KeyboardInterrupt:
        # kos.process_manager.stop_kclip("jiggle")
        # Cleanup: zero position and disable torque
        command = {'actuator_id': motor_id, 'position': 0}
        kos.actuator.command_actuators(commands=[command])
        kos.actuator.configure_actuator(
            actuator_id=motor_id, 
            torque_enabled=False
        )
        kos.process_manager.stop_kclip()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run motor jiggle test')
    parser.add_argument('--motor_id', type=int, default=41, help='Motor ID')
    parser.add_argument('--kp', type=float, default=120.0, help='Proportional gain')
    parser.add_argument('--kd', type=float, default=10.0, help='Derivative gain')
    parser.add_argument('--period', type=float, default=10.0, help='Oscillation period (seconds)')
    parser.add_argument('--amplitude', type=float, default=np.pi/2, help='Oscillation amplitude (radians)')
    parser.add_argument('--max_torque', type=float, default=40.0, help='Maximum torque')
    args = parser.parse_args()

    main(
        motor_id=args.motor_id, 
        kp=args.kp, 
        kd=args.kd, 
        period=args.period, 
        amplitude=args.amplitude, 
        max_torque=args.max_torque
    )
