import sys
import termios
import tty

import pykos


class ComplianceController:
    def __init__(self, zero_on_start: bool = False) -> None:
        self.kos = pykos.KOS()

        self.kos.process_manager.start_kclip("posing")

        self.left_arm_ids = [11, 12, 13, 14, 15]
        self.right_arm_ids = [21, 22, 23, 24, 25]
        self.left_leg_ids = [31, 32, 33, 34, 35]
        self.right_leg_ids = [41, 42, 43, 44, 45]

        # Group IDs for easier control
        self.all_arms = self.left_arm_ids + self.right_arm_ids
        self.all_legs = self.left_leg_ids + self.right_leg_ids

        self.stiff_gains = {
            'type_four': {'kp': 120, 'kd': 10},  # hip and knee
            'type_three': {'kp': 60, 'kd': 5},   # middle joints
            'type_two': {'kp': 17, 'kd': 5},     # ankle
            'type_zero': {'kp': 10, 'kd': 1},    # gripper
        }

        # Lower gains for compliant mode
        self.compliant_gains = {
            'type_four': {'kp': 20, 'kd': 2},
            'type_three': {'kp': 10, 'kd': 1},
            'type_two': {'kp': 5, 'kd': 1},
            'type_zero': {'kp': 10, 'kd': 1},
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

        # Initialize all motors with stiff gains
        self.configure_arms(stiff=True)
        self.configure_legs(stiff=True)

    def configure_legs(self, stiff: bool = True) -> None:
        gains = self.stiff_gains if stiff else self.compliant_gains

        for id in self.all_legs:
            if id % 10 in [1, 4]:  # type four (hip and knee)
                self.kos.actuator.configure_actuator(
                    actuator_id=id,
                    kp=gains['type_four']['kp'],
                    kd=gains['type_four']['kd'],
                    torque_enabled=True
                )
            elif id % 10 in [2, 3]:  # type three (middle joints)
                self.kos.actuator.configure_actuator(
                    actuator_id=id,
                    kp=gains['type_three']['kp'],
                    kd=gains['type_three']['kd'],
                    torque_enabled=True
                )
            elif id % 10 == 5:  # type two (ankle)
                self.kos.actuator.configure_actuator(
                    actuator_id=id,
                    kp=gains['type_two']['kp'],
                    kd=gains['type_two']['kd'],
                    torque_enabled=True
                )

    def configure_arms(self, stiff: bool = True) -> None:
        gains = self.stiff_gains if stiff else self.compliant_gains

        for id in self.all_arms:
            if id % 10 in [1, 2]:  # type three (shoulder)
                self.kos.actuator.configure_actuator(
                    actuator_id=id,
                    kp=gains['type_three']['kp'],
                    kd=gains['type_three']['kd'],
                    torque_enabled=True
                )
            elif id % 10 in [3, 4, 5]:  # type two (elbow and wrist)
                self.kos.actuator.configure_actuator(
                    actuator_id=id,
                    kp=gains['type_two']['kp'],
                    kd=gains['type_two']['kd'],
                    torque_enabled=True
                )
            elif id % 10 == 6:  # type zero (gripper)
                self.kos.actuator.configure_actuator(
                    actuator_id=id,
                    kp=gains['type_zero']['kp'],
                    kd=gains['type_zero']['kd'],
                    torque_enabled=True
                )

    def toggle_arms(self) -> None:
        self.arms_compliant = not self.arms_compliant
        self.configure_arms(stiff=not self.arms_compliant)
        print(f"Arms are now {'compliant' if self.arms_compliant else 'stiff'}")

    def toggle_legs(self) -> None:
        self.legs_compliant = not self.legs_compliant
        self.configure_legs(stiff=not self.legs_compliant)
        print(f"Legs are now {'compliant' if self.legs_compliant else 'stiff'}")

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
    controller = ComplianceController()
    print("\nCompliance Toggle Controls:")
    print("'a' - Toggle arms compliance")
    print("'l' - Toggle legs compliance")
    print("'q' - Quit")

    try:
        while True:
            char = get_char()

            if char == 'a':
                controller.toggle_arms()
            elif char == 'l':
                controller.toggle_legs()
            elif char == 'q':
                print("\nExiting...")
                break

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Disable torque on exit
        for id in controller.all_arms + controller.all_legs:
            controller.kos.actuator.configure_actuator(
                actuator_id=id,
                torque_enabled=False
            )
        controller.kos.process_manager.stop_kclip()
        controller.kos.close()

if __name__ == "__main__":
    main()
