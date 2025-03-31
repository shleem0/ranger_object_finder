import time
import adafruit_lsm303_accel
import board

i2c = board.I2C()
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c)

def read_angle():

    while True:
        acc_x, acc_y = sensor.acceleration

        print(acc_x, acc_y)
        time.sleep(0.2)

read_angle()

