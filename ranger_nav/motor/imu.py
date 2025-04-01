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
current_val &= ~(0b11 << 2)
current_val |= (0b10 << 2)
bus.write_byte_data(I2C, CTRL1_XL, current_val)

current_val = bus.read_byte_data(I2C, CTRL2_G)
current_val &= ~(0b11 << 2)
current_val |= (0b01 << 2)
bus.write_byte_data(I2C, CTRL1_XL, current_val)


def read_angle():

    with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/imu_data.txt","w") as f:

        f.write("0.0 0.0 0.0")


    while True:

        ax, ay, az, gx, gy, gz = sensor.get_readings()

        if ax < 0:
            ax += 0.004
        elif ax > 0:
            ax -= 0.004

        if ay < 0:
            ay += 0.004
        elif ay > 0:
            ay -= 0.004

        if angular_vel < 0:
            angular_vel += 10
        elif angular_vel > 0:
            angular_vel -= 10
        
        accel_x = ax * 0.122 * 9.80665 / 1000
        accel_y = ay * 0.122 * 9.80665 / 1000
        angular_vel = gz * 8.75 * (pi / 180) / 1000




        with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/imu_data.txt","w") as f:

            f.write(f"{accel_x} {accel_y} {angular_vel}")

        time.sleep(0.2)


read_angle()
