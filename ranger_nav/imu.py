from LSM6DS3 import *
import time

sensor = LSM6DS3(ACC_ODR=ACC_ODR_1_66_KHz,
                 GYRO_ODR=GYRO_ODR_1_66_KHS,
                 enable_acc=ENABLE_ACC_ALL_AXIS,
                 enable_gyro=ENABLE_GYRO_ALL_AXIS,
                 acc_interrupt=False,
                 gyro_interrupt=False,
                 acc_scale=ACC_SCALE_16G,
                 gyro_scale=GYRO_SCALE_2000DPS)
def read_angle():

    while True:

        print(sensor.getAccData())
        time.sleep(0.2)

read_angle()

