import pykos
import time

kos = pykos.KOS()

all_ids = [1, 2, 3, 4, 5, 6,
           11, 12, 13, 14, 15, 16,
           21, 22, 23, 24, 25,
           31, 32, 33, 34, 35]

for id in all_ids:
    print(f"Configuring actuator {id}")
    kos.actuator.configure_actuator(actuator_id=id, kp=10, kd=5, torque_enabled=True, zero_position=True)

counter = 0
dir = 1

while True:
    commands = [{'actuator_id': id, 'position': counter} for id in all_ids]
    kos.actuator.command_actuators(commands=commands)

    if counter > 20:
        dir = -1
    elif counter < -20:
        dir = 1
        
    counter += dir
    print(counter)

    time.sleep(0.01)
