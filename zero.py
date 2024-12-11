import time
import pykos

# all_ids = [11, 12, 13, 14, 15, 16,
#            21, 22, 23, 24, 25, 26,
#            31, 32, 33, 34, 35,
#            41, 42, 43, 44, 45]

all_ids = [
           31, 32, 33, 34, 35,
           41, 42, 43, 44, 45
           ]

kos = pykos.KOS()

for id in all_ids:
    print(f"Zeroing motor {id}...")
    kos.actuator.configure_actuator(actuator_id=id, kp=1, kd=0.1, torque_enabled=False, zero_position=True)
    time.sleep(0.1)

print("\nAll motors zeroed!\n")
