import time
import pykos

kos = pykos.KOS()

id=42

# new_id = 31
# kos.actuator.configure_actuator(actuator_id=id, kp=10, kd=1, torque_enabled=False, new_actuator_id=new_id)

kos.actuator.configure_actuator(actuator_id=id, kp=10, kd=0.1, torque_enabled=True, zero_position=True)

counter = 0
dir = 1
while True:
    # Command all actuators to position 0
    commands = [{'actuator_id': id, 'position': counter}]
    kos.actuator.command_actuators(commands=commands)
    time.sleep(0.1)
    counter += dir
    break

    if counter > 10:
        dir = -1
    elif counter < 0:
        dir = 1


# Change id
id = 31

# kos.actuator.configure_actuator(actuator_id=id)
