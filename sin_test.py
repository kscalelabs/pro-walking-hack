import pykos
import argparse
import numpy as np
import time

kos = pykos.KOS()


def main(motor_id=11, kp=1.0, kd=0.1, period=1.0, amplitude=np.pi/4):
    try:
        try:
            kos.process_manager.start_kclip("jiggle")
        except Exception as e:
            print(e)
        # Configure the actuator first
        print(f"Configuring actuator {motor_id}")
        kos.actuator.configure_actuator(
            actuator_id=motor_id, 
            kp=kp, 
            kd=kd, 
            max_torque=10.0,
            torque_enabled=True, 
            zero_position=True
        )

        start_time = time.time()
        while True:
            # Calculate sine wave position: amplitude * sin(2π * t/T)
            t = time.time() - start_time
            position = amplitude * np.sin(2 * np.pi * t / period) * 180 / np.pi
            
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run motor jiggle test')
    parser.add_argument('--motor_id', type=int, default=41, help='Motor ID')
    parser.add_argument('--kp', type=float, default=10.0, help='Proportional gain')
    parser.add_argument('--kd', type=float, default=1, help='Derivative gain')
    parser.add_argument('--period', type=float, default=10.0, help='Oscillation period (seconds)')
    parser.add_argument('--amplitude', type=float, default=np.pi/4, help='Oscillation amplitude (radians)')
    
    args = parser.parse_args()
    main(motor_id=args.motor_id, kp=args.kp, kd=args.kd, period=args.period, amplitude=args.amplitude)
