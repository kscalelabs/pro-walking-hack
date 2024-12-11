import time
import pykos
import math

kos = pykos.KOS()

all_ids = [11, 12, 13, 14, 15, 16] + [21, 22, 23, 24, 25, 26] + [31, 32, 33, 34, 35] + [41, 42, 43, 44, 45]

for id in all_ids:
    kos.actuator.configure_actuator(actuator_id=id, kp=10, kd=0.1, torque_enabled=True)

# Motion parameters
amplitude = 2.0  # degrees
period = 5.0     # seconds
phase_shift = 360 / len(all_ids)  # degrees, evenly distributed across motors

freq = 100  # hz
t_start = time.time()
while True:
    t = time.time() - t_start
    commands = [
        {
            'actuator_id': id,
            'position': amplitude * math.sin(math.radians(360 * t / period + phase_shift * i))
        } 
        for i, id in enumerate(all_ids)
    ]
    kos.actuator.command_actuators(commands=commands)
    time.sleep(1/freq)
