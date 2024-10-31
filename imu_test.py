"""Script to graph the Hexmove IMU."""

import argparse
import time

import matplotlib.pyplot as plt

from imu import HexmoveImuReader

"""
       sudo ip link set can0 up type can bitrate 500000
       ip link show can0
"""

def main() -> None:
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="IMU data visualization.")
    parser.add_argument("--buffer", type=int, default=100, help="Length of the buffer.")
    parser.add_argument("--sleep", type=float, default=0.1, help="Time to sleep in seconds.")
    args = parser.parse_args()

    # Initialize the IMU reader
    try:
        imu_reader = HexmoveImuReader("can0", 1, 1)
    except Exception as e:
        print(f"Failed to initialize IMU reader: {e}")
        return

    # Initialize data arrays with the specified length
    angles_x = [0.0] * args.buffer
    angles_y = [0.0] * args.buffer
    angles_z = [0.0] * args.buffer
    velocities_x = [0.0] * args.buffer
    velocities_y = [0.0] * args.buffer
    velocities_z = [0.0] * args.buffer

    time.sleep(1)

    angle_offset = [imu_reader.get_data().x_angle, imu_reader.get_data().y_angle, imu_reader.get_data().z_angle]

    try:
        plt.ion()  # Turn on interactive mode
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))

        while True:
            # Get the current IMU data
            data = imu_reader.get_data()

            angles_x.append(data.x_angle - angle_offset[0])
            angles_y.append(data.y_angle - angle_offset[1])
            angles_z.append(data.z_angle - angle_offset[2])
            velocities_x.append(data.x_velocity)
            velocities_y.append(data.y_velocity)
            velocities_z.append(data.z_velocity)

            angles_x.pop(0)
            angles_y.pop(0)
            angles_z.pop(0)
            velocities_x.pop(0)
            velocities_y.pop(0)
            velocities_z.pop(0)

            # Clear previous plots
            axs[0].cla()
            axs[1].cla()

            # Plot angles
            axs[0].plot(angles_x, label="X Angle")
            axs[0].plot(angles_y, label="Y Angle")
            axs[0].plot(angles_z, label="Z Angle")
            axs[0].set_title("Angular Position")
            axs[0].legend()

            # Plot velocities
            axs[1].plot(velocities_x, label="X Velocity")
            axs[1].plot(velocities_y, label="Y Velocity")
            axs[1].plot(velocities_z, label="Z Velocity")
            axs[1].set_title("Angular Velocity")
            axs[1].legend()

            plt.pause(args.sleep)

            time.sleep(args.sleep)

    except KeyboardInterrupt:
        print("Stopping IMU reader...")
    finally:
        imu_reader.stop()
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Show the final plot


if __name__ == "__main__":
    main()
