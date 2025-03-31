import smbus
import time
from mpu6050 import mpu6050

sensor = mpu6050(0x6A)
bus = smbus.SMBus(1)

def read_angle():

    while True:
        a_data = sensor.get_accel_data()
        gyro = sensor.get_gyro_data()

        print(a_data['x'], a_data['y'])
        time.sleep(0.2)

read_angle()

