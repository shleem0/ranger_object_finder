sudo python3 motor/encoder.py & 
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p odom_frame:=odom --ros-args -p base_frame:=base_link --ros-args -p map_frame:=map --ros-args -p scan_topic:=/scan --ros-args -p map_update_interval:=0.1 --ros-args -p max_laser_range:=3.5 --ros-args -p minimum_travel_distance:=0.1 --ros-args -p use_scan_matching:=true --ros-args -p minimum_travel_heading:=1.57 --ros-args -p do_loop_closing:=true &
ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py & 
ros2 run movement_controller odom_publisher & 
ros2 launch nav2_bringup navigation_launch.py
