import smbus
import time

AS5600_ADDR = 0x36
AS5600_ANG = 0x0C

bus = smbus.SMBus(1)

def read_angle():

    while True:
        raw_data = bus.read_i2c_block_data(AS5600_ADDR, AS5600_ANG, 2)
        angle = (raw_data[0] << 8) + raw_data[1]

        angle_deg = (angle / 4096) * 360
        print(angle_deg)
        time.sleep(0.1)

read_angle()

