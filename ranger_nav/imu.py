from machine import I2C
import lsm6ds3 import LSM6DS3, NORMAL_MODE_104HZ
import time

i2c = I2C(0, scl=13, sda=12)
sensor = LSM6DS3(i2c, mode=NORMAL_MODE_104HZ)

def read_angle():

    while True:
        acc_x, acc_y, acc_z, gx, gy, gz = sensor.get_readings()

        print(acc_x, acc_y)
        time.sleep(0.2)

read_angle()

