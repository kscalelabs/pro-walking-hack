import numpy as np
import matplotlib.pyplot as plt

# Load the data from the text file
data = np.loadtxt('imu_ang_euler_log.txt', delimiter=',', skiprows=1)

# Assuming the data has three columns, extract them
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

# Create a plot
plt.figure(figsize=(10, 6))
plt.plot(x, label='X-angle')
plt.plot(y, label='Y-angle')
plt.plot(z, label='Z-angle')

# Add labels and title
plt.xlabel('Time (steps)')
plt.ylabel('Angle (rad)')
plt.title('IMU Angular Euler Data')
plt.legend()

# Save the plot as a PNG file
plt.savefig('imu_ang_euler_plot.png')

# Show the plot
plt.show()
