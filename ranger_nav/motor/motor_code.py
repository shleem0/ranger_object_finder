import time
import sys
from grove.grove_i2c_motor_driver import MotorDriver
from grove.grove_optical_rotary_encoder import GroveOpticalRotaryEncoder
from multiprocessing import Queue


motor = MotorDriver()

while True:
    command = input("enter your command:" )

    if command == 'w': 
        motor.set_dir(True,True)
        motor.set_speed(50,50)

    elif command == "s":
        motor.set_dir(False,False)
        motor.set_speed(50,50)

    elif command== 'r':
        motor.set_dir(True,True)
        motor.set_speed(100,100)

    elif command == 'q':
        motor.set_dir(False,False)
        motor.set_speed(100,100)

    elif command == 'd' :
        motor.set_dir(False,True)
        motor.set_speed(100,100)

    elif command == 'a' :
        motor.set_dir(True, False)
        motor.set_speed(100,100)

    elif command == '' :
        motor.set_speed(0,0)





