from lsm6ds3 import LSM6DS3
import smbus
import time
from math import pi

I2C = 0x6A
CTRL1_XL = 0x10
CTRL2_G = 0x11

sensor = LSM6DS3()
bus = smbus.SMBus(1)

#setting sensitivities
current_val = bus.read_byte_data(I2C, CTRL1_XL)
new_val = (current_val & 0xE7)
bus.write_byte_data(I2C, CTRL1_XL, new_val)

current_val = bus.read_byte_data(I2C, CTRL2_G)
new_val = (current_val & 0xE7)
bus.write_byte_data(I2C, CTRL2_G, new_val)


def read_angle():

    while True:

        ax, ay, az, gx, gy, gz = sensor.get_readings()
        
        accel_x = ax * 0.061 * 9.80665 / 1000
        accel_y = ay * 0.061 * 9.80665 / 1000
        angular_vel = gz * 0.00875 * (pi / 180)
        
        with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/imu_data.txt","w") as f:

            f.write(accel_x, accel_y, angular_vel)

        time.sleep(0.2)


read_angle()
