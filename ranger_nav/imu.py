import smbus
import time

AS5600_ADDR = 0x36
AS5600_ANG_LSB = 0x0C
AS5600_ANG_MSB = 0x0D
bus = smbus.SMBus(1)

def read_angle():

    while True:
        raw_lsb = bus.read_byte_data(AS5600_ADDR, AS5600_ANG_LSB)
        raw_msb = bus.read_byte_data(AS5600_ADDR, AS5600_ANG_LSB)

        angle = (raw_msb << 8) | raw_lsb
        print(angle)

        angle_deg = (angle / 4096) * 360
        print(angle_deg)
        time.sleep(0.1)

read_angle()

