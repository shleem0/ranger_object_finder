import time
import sys

from grove.grove_i2c_motor_driver import MotorDriver
from grove.grove_optical_rotary_encoder import GroveOpticalRotaryEncoder

motor = MotorDriver()
PIN1 = 16
PIN2 = 5
encoder1 = GroveOpticalRotaryEncoder(PIN1)
encoder2 = GroveOpticalRotaryEncoder(PIN2)
def track_position():
    while True:
        position2 = encoder2.position()
        f2 = open("motor_data2.txt","w")
        position1 = encoder1.position()
        f1 = open("motor_data1.txt","w")
        f1.write(f"{position1}")
        f1.flush()

        f2.write(f"{position2}")
        f2.flush()
        print(f"\rMotor1 Position: {position1} degrees and Motor2 Position:{position2} degrees ",file=sys.stderr,end='')
        time.sleep(0.1)
   
track_position()
