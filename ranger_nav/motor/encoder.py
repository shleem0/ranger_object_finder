import time
import sys
import threading

from grove.grove_i2c_motor_driver import MotorDriver
from grove.grove_optical_rotary_encoder import GroveOpticalRotaryEncoder


motor = MotorDriver()
PIN1 = 16
PIN2 = 5
encoder1 = GroveOpticalRotaryEncoder(PIN1)
encoder2 = GroveOpticalRotaryEncoder(PIN2)

def track_position():
    with open("motor_data1.txt","w") as f1, open("motor_data2.txt","w") as f2:
        threading.Timer(0.1, track_position).start()
        
        position1 = encoder1.position()
        f1.seek(0)      
        f1.write(f"{position1}")
        f1.flush()

        position2 = encoder2.position()
        f2.seek(0)
        f2.write(f"{position2}")
        f2.flush()

        print(f"{position1}, {position2}")

   
track_position()
