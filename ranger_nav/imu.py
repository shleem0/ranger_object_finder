from lsm6ds3 import LSM6DS3
import time

sensor = LSM6DS3()
def read_angle():

    while True:

        ax, ay, az, gx, gy, gz = sensor.get_readings()
        print(ax, ay)
        time.sleep(0.2)

read_angle()

