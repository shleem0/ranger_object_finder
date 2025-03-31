import smbus
import time
from grove.grove_6_axis_accel_gyro_bmi088 import GroveAccelGyroBMI088

sensor = GroveAccelGyroBMI088()
bus = smbus.SMBus(1)

def read_angle():

    while True:
        a_data = sensor.get_accel()

        print(a_data['x'], a_data['y'])
        time.sleep(0.2)

read_angle()

