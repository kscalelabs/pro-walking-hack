import sys
import termios
import tty
import threading
import time
import json
from datetime import datetime
import argparse

import pykos


class ComplianceController:
    def __init__(self, zero_on_start: bool = False, position_file: str = None) -> None:
        self.kos = pykos.KOS()

        self.kos.process_manager.start_kclip("posing")

        self.left_arm_ids = [11, 12, 13, 14, 15]
        self.right_arm_ids = [21, 22, 23, 24] #5]
        self.left_leg_ids = [31, 32, 33, 34, 35]
        self.right_leg_ids = [41, 42, 43, 44, 45]

        # Group IDs for easier control
        self.all_arms = self.left_arm_ids + self.right_arm_ids
        self.all_legs = self.left_leg_ids + self.right_leg_ids

        self.stiff_params = {
            'type_four': {'kp': 220, 'kd': 10, 'torque_limit': 20},  # hip and knee
            'type_three': {'kp': 100, 'kd': 5, 'torque_limit': 20},   # middle joints
            'type_two': {'kp': 30, 'kd': 5, 'torque_limit': 17},     # ankle
            'type_zero': {'kp': 10, 'kd': 1, 'torque_limit': 10},    # gripper
        }

        # Lower gains for compliant mode
        self.compliant_params = {
            'type_four': {'kp': 30, 'kd': 2, 'torque_limit': 20},
            'type_three': {'kp': 20, 'kd': 1, 'torque_limit': 20},
            'type_two': {'kp': 15, 'kd': 1, 'torque_limit': 20},
            'type_zero': {'kp': 10, 'kd': 1, 'torque_limit': 20},
        }

        # Track compliance state
        self.arms_compliant = False
        self.legs_compliant = False

        if zero_on_start:
            for id in self.all_arms + self.all_legs:
                self.kos.actuator.configure_actuator(
                    actuator_id=id,
                    kp=10,
                    kd=1,
                    zero_position=True
                )
        elif position_file:  # Load position if file provided
            try:
                with open(position_file, 'r') as f:
                    positions = json.load(f)
                
                print(f"Loading positions from {position_file}")
                # Configure actuators with moderate gains
                for id in self.all_arms + self.all_legs:
                    self.kos.actuator.configure_actuator(
                        actuator_id=id,
                        kp=50,
                        kd=2,
                        torque_enabled=True
                    )
                
                # Command positions
                commands = [
                    {'actuator_id': int(id), 'position': float(pos)}
                    for id, pos in positions.items()
                ]
                self.kos.actuator.command_actuators(commands=commands)
                
                # Wait for movement to complete
                time.sleep(2.0)
            except Exception as e:
                print(f"Error loading positions: {e}")

        self._kos_lock = threading.Lock()    # Lock for KOS access
        self._state_lock = threading.Lock()  # Lock for compliance state

        # Initialize all motors with stiff gains
        self.configure_arms(stiff=True)
        self.configure_legs(stiff=True)

        # Add thread control flags and locks
        self._running = False
        self._control_thread = None

        # Add position recording functionality
        self._recorded_positions = {}

    def configure_legs(self, stiff: bool = True) -> None:
        params = self.stiff_params if stiff else self.compliant_params
        
        with self._kos_lock:
            for id in self.all_legs:
                if id % 10 in [1, 4]:  # type four (hip and knee)
                    self.kos.actuator.configure_actuator(
                        actuator_id=id,
                        kp=params['type_four']['kp'],
                        kd=params['type_four']['kd'],
                        max_torque=params['type_four']['torque_limit'],
                        torque_enabled=True
                    )
                elif id % 10 in [2, 3]:  # type three (middle joints)
                    self.kos.actuator.configure_actuator(
                        actuator_id=id,
                        kp=params['type_three']['kp'],
                        kd=params['type_three']['kd'],
                        max_torque=params['type_three']['torque_limit'],
                        torque_enabled=True
                    )
                elif id % 10 == 5:  # type two (ankle)
                    self.kos.actuator.configure_actuator(
                        actuator_id=id,
                        kp=params['type_two']['kp'],
                        kd=params['type_two']['kd'],
                        max_torque=params['type_two']['torque_limit'],
                        torque_enabled=True
                    )

    def configure_arms(self, stiff: bool = True) -> None:
        params = self.stiff_params if stiff else self.compliant_params
        
        with self._kos_lock:
            for id in self.all_arms:
                if id % 10 in [1, 2]:  # type three (shoulder)
                    self.kos.actuator.configure_actuator(
                        actuator_id=id,
                        kp=params['type_three']['kp'],
                        kd=params['type_three']['kd'],
                        max_torque=params['type_three']['torque_limit'],
                        torque_enabled=True
                    )
                elif id % 10 in [3, 4, 5]:  # type two (elbow and wrist)
                    self.kos.actuator.configure_actuator(
                        actuator_id=id,
                        kp=params['type_two']['kp'],
                        kd=params['type_two']['kd'],
                        max_torque=params['type_two']['torque_limit'],
                        torque_enabled=True
                    )
                elif id % 10 == 6:  # type zero (gripper)
                    self.kos.actuator.configure_actuator(
                        actuator_id=id,
                        kp=params['type_zero']['kp'],
                        kd=params['type_zero']['kd'],
                        max_torque=params['type_zero']['torque_limit'],
                        torque_enabled=True
                    )

    def toggle_arms(self) -> None:
        with self._state_lock:
            self.arms_compliant = not self.arms_compliant
            self.configure_arms(stiff=not self.arms_compliant)
            print(f"Arms are now {'compliant' if self.arms_compliant else 'stiff'}")

    def toggle_legs(self) -> None:
        with self._state_lock:
            self.legs_compliant = not self.legs_compliant
            self.configure_legs(stiff=not self.legs_compliant)
            print(f"Legs are now {'compliant' if self.legs_compliant else 'stiff'}")

    def start_control_thread(self):
        """Start the control thread if not already running"""
        if not self._running:
            self._running = True
            self._control_thread = threading.Thread(target=self._control_loop)
            self._control_thread.daemon = True  # Thread will exit when main program exits
            self._control_thread.start()

    def stop_control_thread(self):
        """Stop the control thread"""
        self._running = False
        if self._control_thread:
            self._control_thread.join()
            self._control_thread = None

    def _control_loop(self):
        """Control loop running in separate thread"""
        while self._running:
            # Safely check compliance state
            with self._state_lock:
                arms_compliant = self.arms_compliant
                legs_compliant = self.legs_compliant

            # Update setpoints if any limbs are in compliant mode
            if arms_compliant:
                with self._kos_lock:
                    states = self.kos.actuator.get_actuators_state(actuator_ids=self.all_arms)
                    commands = [{'actuator_id': state.actuator_id, 'position': state.position} 
                              for state in states]
                    self.kos.actuator.command_actuators(commands=commands)
                
            if legs_compliant:
                with self._kos_lock:
                    states = self.kos.actuator.get_actuators_state(actuator_ids=self.all_legs)
                    commands = [{'actuator_id': state.actuator_id, 'position': state.position} 
                              for state in states]
                    self.kos.actuator.command_actuators(commands=commands)

            time.sleep(0.01)  # Small delay to prevent CPU overload

    def record_positions(self) -> None:
        """Record current positions of all actuators"""
        with self._kos_lock:
            # Get all current positions
            arm_states = self.kos.actuator.get_actuators_state(actuator_ids=self.all_arms)
            leg_states = self.kos.actuator.get_actuators_state(actuator_ids=self.all_legs)
            
            # Create position dictionary
            positions = {
                state.actuator_id: state.position 
                for state in arm_states + leg_states
            }
            
            # Generate timestamp for filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recorded_positions_{timestamp}.json"
            
            # Save to file
            with open(filename, 'w') as f:
                json.dump(positions, f, indent=2)
            
            print(f"\nPositions recorded to {filename}")

def get_char() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main() -> None:
    # Add argument parsing
    parser = argparse.ArgumentParser(description='Robot pose controller')
    parser.add_argument('--zero', action='store_true', help='Zero all positions on start')
    parser.add_argument('--load', type=str, help='Load positions from file on start')
    args = parser.parse_args()

    if args.zero and args.load:
        print("Error: Cannot specify both --zero and --load")
        return

    controller = ComplianceController(
        zero_on_start=args.zero,
        position_file=args.load
    )
    controller.start_control_thread()

    print("\nCompliance Toggle Controls:")
    print("'a' - Toggle arms compliance")
    print("'l' - Toggle legs compliance")
    print("'r' - Record current positions")
    print("'q' - Quit")

    try:
        while True:
            char = get_char()
            if char == 'a':
                controller.toggle_arms()
            elif char == 'l':
                controller.toggle_legs()
            elif char == 'r':
                controller.record_positions()
            elif char == 'q':
                print("\nExiting...")
                break

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        controller.stop_control_thread()
        # Disable torque on exit
        with controller._kos_lock:  # Protect final KOS access
            for id in controller.all_arms + controller.all_legs:
                controller.kos.actuator.configure_actuator(
                    actuator_id=id,
                    torque_enabled=False
                )
            controller.kos.process_manager.stop_kclip()
            controller.kos.close()

if __name__ == "__main__":
    main()
