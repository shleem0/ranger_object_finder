import time
from grove.grove_i2c_motor_driver import MotorDriver
from grove.grove_optical_rotary_encoder import GroveOpticalRotaryEncoder

motor = MotorDriver()
print("motor defined")

while True:
    
    f1 = open("motor/motor_input1.txt", "r")
    f2 = open("motor/motor_input2.txt", "r")

    try:
        left_dir = bool(f1.read().split(" ")[0])
        right_dir = bool(f2.read().split(" ")[0])
    except:
        left_dir = True
        right_dir = True

    try:
        left_v = int(f1.read().split(" ")[1])
        right_v = int(f2.read().split(" ")[1])
    except:
        left_v = 0
        right_v = 0

    
    print(type(left_dir))
    print(left_dir)

    print(type(right_dir))
    print(right_dir)

    motor.set_dir(left_dir, right_dir)
    motor.set_speed(left_v, right_v)
    print("speed set")
    print(f"Motor going at {left_v}:{left_dir}, {right_v}:{right_dir}")
    time.sleep(0.1)

    f1.close()
    f2.close()





