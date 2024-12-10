import pykos

kos = pykos.KOS()

left_leg_ids = [31, 32, 33, 34, 35]
right_leg_ids = [41, 42, 43, 44, 45]

all_ids = left_leg_ids + right_leg_ids

for id in all_ids:
    kos.actuator.configure_actuator(actuator_id=id, kp=1, kd=1, torque_enabled=True)

# Command all actuators to position 0
commands = [{'actuator_id': id, 'position': 0} for id in all_ids]
kos.actuator.command_actuators(commands=commands)
