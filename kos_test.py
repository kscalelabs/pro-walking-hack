import pykos
import time

kos = pykos.KOS()

all_ids = [11, 12, 13, 14, 15,
           21, 22, 23, 24, 25,
           31, 32, 33, 34, 35,
           41, 42, 43, 44, 45]

for id in all_ids:
    print(f"Configuring actuator {id}")
    kos.actuator.configure_actuator(actuator_id=id, kp=10, kd=5, torque_enabled=True, zero_position=True)

counter = 0
dir = 1

print("Starting loop")
while True:
    # commands = [{'actuator_id': id, 'position': counter} for id in all_ids]
    # kos.actuator.command_actuators(commands=commands)

    # if counter > 20:
    #     dir = -1
    # elif counter < -20:
    #     dir = 1
        
    # counter += dir
    # print(counter)

    time.sleep(0.01)
