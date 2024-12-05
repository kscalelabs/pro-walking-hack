import time
from pykos import KOS

def main():
    kos = KOS()
    actuator_ids = [1, 2, 3, 4, 5, 6, 11, 12, 13, 14, 15, 16]
    prev_states = None
    update_count = 0
    start_time = time.time()
    duration = 5  # Measure frequency over 5 seconds

    while True:
        # Get current actuator states
        current_states = kos.actuator.get_actuators_state(actuator_ids)

        # Check if every value updates since last time
        if prev_states is not None:
            all_updated = all(
                current != previous for current, previous in zip(current_states, prev_states)
            )
            if all_updated:
                update_count += 1

        # Update the previous states
        prev_states = current_states

        # Check if the measurement duration has elapsed
        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            print(f"Update frequency (all values updated): {update_count / duration} Hz")
            update_count = 0  # Reset the update counter
            start_time = time.time()  # Reset start time

        # Sleep for 4ms
        time.sleep(0.004)

if __name__ == "__main__":
    main()
