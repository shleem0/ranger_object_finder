import time
from grove.grove_i2c_motor_driver import MotorDriver
from grove.grove_optical_rotary_encoder import GroveOpticalRotaryEncoder

motor = MotorDriver()

def control_motors():

    with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_input1.txt", "w") as f1, open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_input2.txt", "w") as f2:

        f1.write("True 0")
        f2.write("True 0")

    with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_input1.txt", "r") as f1, open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_input2.txt", "r") as f2:

        while True:
            
            file1 = f1.read().split()
            file2 = f2.read().split()

            try:
                left_dir = eval(file1[0])
                right_dir = eval(file2[0])
            except:
                left_dir = True
                right_dir = True

            try:
                left_v = int(file1[1])
                right_v = int(file2[1])
            except:
                left_v = 0
                right_v = 0

            motor.set_dir(left_dir, right_dir)
            motor.set_speed(left_v, right_v)

control_motors()




