import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from tf_transformations import quaternion_from_euler

from math import sin, cos, pi, sqrt
import numpy as np

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        self.broadcaster = TransformBroadcaster(self)

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.map_data = None
        self.goal = None
        
        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_motor_pos1 = 0.0
        self.prev_motor_pos2 = 0.0

        self.last_time = self.get_clock().now()

        # Create a timer to publish at a fixed rate (e.g., every 0.1 seconds)
        self.trans_timer = self.create_timer(0.2, self.timer_callback)
        self.pos_timer = self.create_timer(5, self.print_pos)
        self.goal_pub_timer = self.create_timer(5, self.publish_goal_pose)


    def print_pos(self):
            print(f"Current pos:\nx: {self.x}, y: {self.y}, angle: {self.theta}\n")


    def timer_callback(self):
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds

        f1 = open("motor/motor_data1.txt", "r")
        f2 = open("motor/motor_data2.txt", "r")

        try:
            motor_pos1 = float(f1.read())
            motor_pos2 = float(f2.read())
        except:
            motor_pos1 = 0.0
            motor_pos2 = 0.0

        f1.close()
        f2.close()

        angle_dif1 = (motor_pos1 - self.prev_motor_pos1) * pi / 180
        angle_dif2 = (motor_pos2 - self.prev_motor_pos2) * pi / 180

        vel1 = 0.08 * angle_dif1 / 0.1
        vel2 = 0.08 * angle_dif2 / 0.1

        linear_velocity = (vel1 + vel2) / 2 #m/s (example value)
        angular_velocity = (vel1 - vel2) / 0.135  # rad/s (example value)

        # Update position and orientation based on velocity
        self.theta += angular_velocity * dt
        self.x += linear_velocity * dt * cos(self.theta)
        self.y += linear_velocity * dt * sin(self.theta)

        # Convert angle to quaternion for the TF message
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)

        self.prev_motor_pos1 = motor_pos1
        self.prev_motor_pos2 = motor_pos2

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_publisher.publish(odom)


        # Transform from odom to base_link
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = current_time
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'

        # Example values - Replace with actual odometry data
        t_odom_base.transform.translation.x = self.x
        t_odom_base.transform.translation.y = self.y
        t_odom_base.transform.translation.z = 00
        t_odom_base.transform.rotation.x = qx
        t_odom_base.transform.rotation.y = qy
        t_odom_base.transform.rotation.z = qz
        t_odom_base.transform.rotation.w = qw

        # Transform from base_link to base_scan
        t_base_scan = TransformStamped()
        t_base_scan.header.stamp = current_time
        t_base_scan.header.frame_id = 'base_link'
        t_base_scan.child_frame_id = 'laser'

        # Adjust these values based on where your LiDAR is mounted
        t_base_scan.transform.translation.x = 0.0  # 10 cm in front of base_link
        t_base_scan.transform.translation.y = 0.0
        t_base_scan.transform.translation.z = 0.13  # LiDAR is 15 cm above base_link
        t_base_scan.transform.rotation.x = 0.0
        t_base_scan.transform.rotation.y = 0.0
        t_base_scan.transform.rotation.z = 0.0
        t_base_scan.transform.rotation.w = 1.0

        # Publish the transforms
        self.tf_broadcaster.sendTransform(t_odom_base)
        self.tf_broadcaster.sendTransform(t_base_scan)

        # Save current time for the next iteration
        self.last_time = current_time



    def map_callback(self, msg):

        self.map_data = msg

        if self.map_data:
            self.goal = self.find_goal_pose(self.map_data)
        else:
            print("No map")




    def find_goal_pose(self, map_data):
        # Extract the map dimensions and data
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution  # In meters per cell
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        
        # Convert map data (OccupancyGrid) to a numpy array for easier processing
        map_array = np.array(map_data.data).reshape((height, width))

        # Find the edge of the free space (value 0 corresponds to free space in OccupancyGrid)
        edge_points = []
        
        # Check the edges of the map (first and last rows and columns)
        for x in range(width):
            if map_array[0, x] == -1:  # First row
                edge_points.append((x, 0))
            if map_array[height-1, x] == -1:  # Last row
                edge_points.append((x, height-1))
        for y in range(height):
            if map_array[y, 0] == -1:  # First column
                edge_points.append((0, y))
            if map_array[y, width-1] == -1:  # Last column
                edge_points.append((width-1, y))
        
        # If we found any free edge points, return the first one (or any other strategy)
        if edge_points:
            
            edge_point = min(edge_points, key = lambda d: sqrt((self.x - d[0]) * (self.x - d[0]) + (self.y - d[1]) * (self.y - d[1]))) # Choose closest edge point
            
            # Convert map coordinates to world coordinates
            goal_x = origin_x + edge_point[0] * resolution
            goal_y = origin_y + edge_point[1] * resolution
            
            # Create and return the goal pose
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            return goal_pose
        
        return None




    def publish_goal_pose(self):
        if self.goal:
            self.goal_pose_pub.publish(self.goal)
            print (f"Goal pose: x:{self.goal.pose.position.x}, y:{self.goal.pose.position.y}\n")

        else:
            print("No goal pose\n")




    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Robot parameters
        wheelbase = 0.13  # The distance between the two wheels (meters)
        motor_max_rpm = 150

        motor_max_speed = 2 * pi * 0.04 * (motor_max_rpm / 60)
        
        # Calculate left and right motor speeds
        v_left = linear_velocity - (wheelbase / 2) * angular_velocity
        v_right = linear_velocity + (wheelbase / 2) * angular_velocity

        left_dir = True
        right_dir = True

        if v_left < 0:
            left_dir = False
            v_left = -v_left

        if v_right < 0:
            right_dir = False
            v_right = -v_right

        f1 = open("motor/motor_input1.txt", "w")
        f2 = open("motor/motor_input2.txt", "w")

        motor1_speed = int(v_left / motor_max_speed * 100)
        motor2_speed = int(v_right / motor_max_speed * 100)

        f1.write(f"{left_dir} {motor1_speed}")
        f2.write(f"{right_dir} {motor2_speed}")

        f1.close()
        f2.close()


        

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
