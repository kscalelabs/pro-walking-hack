import time
from imu import HexmoveImuReader

imu = HexmoveImuReader("can0", 1, 1)

while True:
    data = imu.get_data()
    print(f"x: {data.x_angle}, y: {data.y_angle}, z: {data.z_angle}")
    time.sleep(0.1)
