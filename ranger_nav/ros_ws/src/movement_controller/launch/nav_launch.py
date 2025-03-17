from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(package = 'movement_controller',
        executable = 'odom_publisher',
        name = 'movement_controller',
        output = 'screen'),

        Node(
        package='ld08_driver',
        executable='ld08_driver',
        name='ld08_driver'),

        ExecuteProcess(
            cmd = ['sudo', 'python3', '/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_code.py']
        ),

        ExecuteProcess(
            cmd = ['sudo', 'python3', '/home/ubuntu/ranger_object_finder/ranger_nav/motor/encoder.py']
        )

    ]
    )
