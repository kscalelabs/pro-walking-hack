import pykos
import argparse
import numpy as np
import time
from demo import get_to_position  # Add this import at the top

"""

Acuator Id Mapping:

Left Leg:

31 - Hip

34 - Knee

35 - Ankle


Right Leg:

41 - Hip

44 - Knee

45 - Ankle

"""

kos = pykos.KOS()

ALL_IDS = [
    # 21, 22, 23, 24, 25, 26, # arm
    31, 32, 33, 34, 35,
    41, 42, 43, 44, 45]

def set_up(kos):
    # arm_ids = [21, 22, 23, 24, 25, 26]
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

    # for id in arm_ids:
    #     print(f"Configuring actuator {id}")
    #     kos.actuator.configure_actuator(actuator_id=id, kp=5, kd=5, max_torque=5, torque_enabled=True)

    state = kos.actuator.get_actuators_state(actuator_ids=ALL_IDS)
    for s in state:
        print(f"Actuator {s.actuator_id} is {s.position:.3f}")

def main(motor_id=42, kp=1.0, kd=0.1, period=1.0, amplitude=np.pi/4, max_torque=40.0):
    try:
        kos.process_manager.start_kclip(f"sine_wave_motor_{motor_id}_kp_{kp}_kd_{kd}_period_{period}_amplitude_{amplitude}_max_torque_{max_torque}")
        set_up(kos)
        
        # Configure the actuator first
        print(f"Configuring actuator {motor_id}")
        kos.actuator.configure_actuator(
            actuator_id=motor_id, 
            kp=kp, 
            kd=kd, 
            max_torque=max_torque,
            torque_enabled=True
        )

        desired_period = 0.01  # 100Hz = 0.01 seconds per loop
        start_time = time.time()
        
        while time.time() - start_time < 15.0:  # Run for exactly 5 seconds
            loop_start = time.time()
            
            # Calculate sine wave position and send command
            t = time.time() - start_time
            position = amplitude * np.sin(2 * np.pi * t / period) * 180 / np.pi
            
            # Command the actuator
            command = {'actuator_id': motor_id, 'position': position}
            kos.actuator.command_actuators(commands=[command])
            
            # Calculate how long to sleep
            computation_time = time.time() - loop_start
            sleep_time = max(0, desired_period - computation_time)
            time.sleep(sleep_time)

        # Get current position for smooth transition
        state = kos.actuator.get_actuators_state(actuator_ids=[motor_id])
        current_pos = state[0].position
        
        # Smoothly return to zero using get_to_position
        get_to_position(kos, motor_id, current_pos, 0, time_period=1, frequency=50)
        
        # Disable torque after completing
        kos.actuator.configure_actuator(
            actuator_id=motor_id, 
            torque_enabled=False
        )
        kos.process_manager.stop_kclip()

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

"""
python3 sine_wave.py --motor_id 44 --kp 120.0 --kd 10.0 --period 10 --max_torque=20 --amplitude 0.52

"""