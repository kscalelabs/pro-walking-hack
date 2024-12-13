""" Demo script for waving. """
import pykos
import time
import numpy as np

ALL_IDS = [
    21, 22, 23, 24, 25, 26, # arm
    31, 32, 33, 34, 35,
    41, 42, 43, 44, 45]


def standing(kos):
    cmd = [{"actuator_id": id, "position": 0} for id in ALL_IDS]
    kos.actuator.command_actuators(commands=cmd)


def get_state(kos, verbose=True):
    state = kos.actuator.get_actuators_state(actuator_ids=ALL_IDS)

    if verbose:
        for s in state:
            print(f"Actuator {s.actuator_id} is {s.position:.3f}")
    
    return state

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


def get_to_position(kos, motor_id, current_position, target_position, time_period, frequency):
    """Set the actuator to the given positon
    from the current position to the target position withing n seconds at f frequency

    Args:
        kos: KOS instance
        motor_id: the motor id to move
        current_position: the current position of the motor
        target_position: the target position of the motor
        time_period: the time period to move the motor
        frequency: the frequency of the movement
    """
    interpolation_factor = np.linspace(current_position, target_position, time_period * frequency)

    for pos in interpolation_factor:
        cmd = [{"actuator_id": motor_id, "position": pos}]
        kos.actuator.command_actuators(commands=cmd)
        time.sleep(1./frequency)


def wave(kos):
    # roll elbow
    roll_id = [23]
    yaw_id = [24]
    gripper_id = [26]

    # roll elbow
    get_to_position(kos, roll_id, 0, -60, 1, 50)
    # yaw elbow
    get_to_position(kos, yaw_id, 0, 90, 1, 50)
    breakpoint()

    time_period = 20
    start_time = time.time()
    while time.time() - start_time < time_period:
        get_to_position(kos, yaw_id, 90, 180, 1, 50)
        get_to_position(kos, gripper_id, 0, 100, 0.5, 50)

        get_to_position(kos, yaw_id, 180, 90, 1, 50)
        get_to_position(kos, gripper_id, 100, 0, 0.5, 50)


if __name__ == "__main__":
    kos = pykos.KOS()
    frequency = 50

    try:
        set_up(kos)
        breakpoint()
        while True:
            wave(kos)

    except KeyboardInterrupt:
        print("Exiting...")
    except RuntimeError as e:
        print(e)
    finally:
        for id in ALL_IDS:
            kos.actuator.configure_actuator(actuator_id=id, torque_enabled=False)
        kos.actuator.command_actuators(commands=[{"actuator_id": id, "position": 0} for id in ALL_IDS])
        print("Torque disabled")
