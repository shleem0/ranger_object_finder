import smbus
import time
from grove.grove_6_axis_acc_compass import lsm303d

sensor = lsm303d.lsm303d()

def read_angle():

    while True:
        a_data = sensor.getAccel()

        print(a_data)
        time.sleep(0.2)

read_angle()

