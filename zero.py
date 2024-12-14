import pykos
import time

all_ids = [11, 12, 13, 14, 15, 16,
            21, 22, 23, 24,# 25, 26,
            31, 32, 33, 34, 35,
            41, 42, 43, 44, 45,
]

kos = pykos.KOS()

for id in all_ids:
    print(f"Zeroing actuator {id}")
    kos.actuator.configure_actuator(
        actuator_id=id,
        kp=10,
        kd=1,
        zero_position=True
    )
    time.sleep(0.05)
