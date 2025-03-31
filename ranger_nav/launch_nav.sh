ros2 run movement_controller odom_publisher & 
ros2 launch slam_toolbox online_async_launch.py &>/dev/null &
ros2 launch nav2_bringup navigation_launch.py &>/dev/null &
ros2 launch ld08_driver ld08.launch.py &
sudo python3 /home/ubuntu/ranger_object_finder/ranger_nav/motor/imu.py & 
sudo python3 /home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_code.py
