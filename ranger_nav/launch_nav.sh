ros2 run movement_controller odom_publisher & 
ros2 launch slam_toolbox online_async_launch.py &>/dev/null &
ros2 launch nav2_bringup navigation_launch.py &>/dev/null &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom &>/dev/null &
ros2 launch ld08_driver ld08.launch.py &
sudo python3 motor/encoder.py & 
sudo python3 motor/motor_code.py
