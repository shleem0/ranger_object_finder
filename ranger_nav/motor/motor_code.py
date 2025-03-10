import time
import sys
from grove.grove_i2c_motor_driver import MotorDriver
from grove.grove_optical_rotary_encoder import GroveOpticalRotaryEncoder


motor = MotorDriver()

while True:
    '''command = input("enter your command:" )

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
        motor.set_speed(0,0)'''
    
    f1 = open("motor_input1.txt", "r")
    f2 = open("motor_input2.txt", "r")

    data_1 = f1.readlines[0].split(" ")
    data_2 = f2.readlines[0].split(" ")

    motor.set_dir(data_1[0], data_2[0])
    motor.set_speed(data_1[1], data_2[1])





