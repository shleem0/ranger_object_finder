import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([

        Node(
            package = "ranger_nav",
            executable =  "ranger_nav",
            name = "ranger_nav_node",
            output = "screen"
        ),

        #IncludeLaunchDescription(
         #   PythonLaunchDescriptionSource(['/afs/inf.ed.ac.uk/user/s22/s2281597/ranger_object_finder/ranger_nav/ros_ws/src/hls_lfcd_lds_driver/launch/hlds_laser.launch.py'])
        #),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(['/afs/inf.ed.ac.uk/user/s22/s2281597/ranger_object_finder/ranger_nav/ros_ws/src/slam_gmapping/slam_gmapping/launch/slam_gmapping.launch.py'])
        #),

    ]
    )

