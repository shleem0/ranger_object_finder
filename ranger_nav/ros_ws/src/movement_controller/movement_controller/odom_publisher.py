import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from tf_transformations import quaternion_from_euler

from math import sin, cos, pi, sqrt
import numpy as np
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_data1.txt", "r") as f1, open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_data2.txt", "r") as f2:
            
            try:
                self.prev_motor_pos1 = float(f1.read())
                self.prev_motor_pos2 = float(f2.read())
            except:
                self.prev_motor_pos1 = 0
                self.prev_motor_pos2 = 0

        self.broadcaster = TransformBroadcaster(self)

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.initial_pose_pub = self.create_publisher(PoseStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.map_data = None
        self.goal = None
        self.published_init = False
        
        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_motor_pos1 = 0.0
        self.prev_motor_pos2 = 0.0

        self.last_time = self.get_clock().now()

        # Create a timer to publish at a fixed rate (e.g., every 0.1 seconds)
        self.trans_timer = self.create_timer(0.2, self.timer_callback)
        self.pos_timer = self.create_timer(10, self.print_pos)
        self.goal_pub_timer = self.create_timer(18, self.publish_goal_pose)


    def print_pos(self):
            print(f"Current pos: x: {self.x}, y: {self.y}, angle: {self.theta}\n")
            if self.goal is not None:
                print(f"Goal pose: x:{self.goal.pose.position.x}, y:{self.goal.pose.position.y}\n")


    def publish_initial_pose(self):

        if self.map_data is None:
            print("No map, not setting initial pose")
        
        else:
            map_width = self.map_data.info.width
            map_height = self.map_data.info.height
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y

            # Compute the center of the map in world coordinates
            center_x = origin_x + (map_width * resolution) / 2
            center_y = origin_y + (map_height * resolution) / 2

            initial_pose = PoseStamped()
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.header.frame_id = "map"  # Frame of reference for the map
                
            # Set position (x, y) and orientation (quaternion)
            initial_pose.pose.position.x = center_x
            initial_pose.pose.position.y = center_y
            initial_pose.pose.position.z = 0.0
                
                # Set orientation using quaternion
            qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)
            initial_pose.pose.orientation.x = qx
            initial_pose.pose.orientation.y = qy
            initial_pose.pose.orientation.z = qz
            initial_pose.pose.orientation.w = qw

            self.x = center_x
            self.y = center_y
            self.theta = 0.0
                
            # Publish the initial pose
            self.initial_pose_pub.publish(initial_pose)
            print(f"Initial pose: {self.x}, {self.y}")




    def timer_callback(self):
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds

        with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_data1.txt", "r") as f1, open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_data2.txt", "r") as f2:

            try:
                motor_pos1 = float(f1.read())
                motor_pos2 = float(f2.read())
            except:
                motor_pos1 = self.prev_motor_pos1
                motor_pos2 = self.prev_motor_pos2

        angle_dif1 = (motor_pos1 - self.prev_motor_pos1) * pi / 180
        angle_dif2 = (motor_pos2 - self.prev_motor_pos2) * pi / 180

        vel1 = 0.04 * (angle_dif1 / dt)
        vel2 = 0.04 * (angle_dif2 / dt)

        linear_velocity = (vel1 + vel2) / 2
        angular_velocity = (vel2 - vel1) / 0.295

        self.theta += angular_velocity * dt
        self.x += linear_velocity * dt * cos(self.theta)
        self.y += linear_velocity * dt * sin(self.theta)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)

        self.prev_motor_pos1 = motor_pos1
        self.prev_motor_pos2 = motor_pos2


        #odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
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
        t_odom_base.header.stamp = current_time.to_msg()
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'

        t_odom_base.transform.translation.x = self.x
        t_odom_base.transform.translation.y = self.y
        t_odom_base.transform.translation.z = 0.0
        t_odom_base.transform.rotation.x = qx
        t_odom_base.transform.rotation.y = qy
        t_odom_base.transform.rotation.z = qz
        t_odom_base.transform.rotation.w = qw


        # Transform from base_link to base_scan
        t_base_scan = TransformStamped()
        t_base_scan.header.stamp = current_time.to_msg()
        t_base_scan.header.frame_id = 'base_link'
        t_base_scan.child_frame_id = 'base_scan'

        t_base_scan.transform.translation.x = 0.0
        t_base_scan.transform.translation.y = 0.0
        t_base_scan.transform.translation.z = 0.13  # LiDAR is 15 cm above base_link
        t_base_scan.transform.rotation.x = 0.0
        t_base_scan.transform.rotation.y = 0.0
        t_base_scan.transform.rotation.z = 0.0
        t_base_scan.transform.rotation.w = 1.0

        # Publish the transforms
        self.broadcaster.sendTransform(t_odom_base)
        self.broadcaster.sendTransform(t_base_scan)

        self.last_time = current_time



    def map_callback(self, msg):
        self.map_data = msg

        if self.published_init == False:
            self.publish_initial_pose()
            self.published_init = True



    def find_goal_pose(self):
        # Extract the self.map dimensions and data
        if self.map_data is not None:

            print("Getting goal pose")
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution  # In meters per cell
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y
            
            # Convert self.map data (OccupancyGrid) to a numpy array for easier processing
            map_array = np.array(self.map_data.data).reshape((height, width))

            # Find the edge of the free space (value 0 corresponds to free space in OccupancyGrid)
            empty_points = []
            
            # Check the edges of the self.map (first and last rows and columns)
            for x in range(width):
                for y in range(height):
                    if map_array[y, x] == 0:
                        empty_points.append((x, y))
    
            # If we found any free edge points, return the first one (or any other strategy)
            if empty_points:
                
                empty_point = max(empty_points, key = lambda d: sqrt((self.x - d[0]) * (self.x - d[0]) + (self.y - d[1]) * (self.y - d[1]))) # Choose furthest explored edge point
                
                # Convert map coordinates to world coordinates
                goal_x = origin_x + empty_point[0] * resolution
                goal_y = origin_y + empty_point[1] * resolution
                
                # Create and return the goal pose
                goal_pose = PoseStamped()
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.header.frame_id = "map"
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.position.z = 0.0
                goal_pose.pose.orientation.w = self.theta
                return goal_pose
        else:
            print("Failed to get goal")
            return None
        


    def publish_goal_pose(self):
        if self.map_data:
            self.goal = self.find_goal_pose()

            if self.goal:
                self.goal_pose_pub.publish(self.goal)

        else:
            print("Cannot get goal pose from map\n")



    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        print(f"Velocity: {linear_velocity}m/s, {angular_velocity}rad/s")

        move = Twist()

        move.angular.z = angular_velocity
        move.linear.x = 0.0
        self.vel_calculation(move)

        time.sleep(1.0)

        #carrying out linear velocity
        move.linear.x = linear_velocity
        move.angular.z = 0.0
        self.vel_calculation(move)

        time.sleep(1.5)

        '''move.linear.x = 0.0
        move.angular.z = 0.0
        self.vel_calculation(move)'''




    def vel_calculation(self, msg):

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Robot parameters
        wheelbase = 0.295  # The distance between the two wheels (meters)
        motor_max_rpm = 150
        motor_max_speed = 2 * pi * 0.04 * (motor_max_rpm / 60)

        robot_weight = 2.2
        weight_factor = 1 + (robot_weight - 1) * 3
        
        # Calculate left and right motor speeds
        v_left = linear_velocity - (wheelbase * angular_velocity) / 2
        v_right = linear_velocity + (wheelbase * angular_velocity) / 2

        left_dir = True
        right_dir = True


        if v_left < 0:
            left_dir = False
            v_left = -v_left

        if v_right < 0:
            right_dir = False
            v_right = -v_right

        motor1_speed = int(min((v_left / motor_max_speed * weight_factor) * 100, 100))
        motor2_speed = int(min((v_right / motor_max_speed * weight_factor) * 100, 100))

        if linear_velocity == 0:
            if angular_velocity > 0:
                motor2_speed = max(75, motor2_speed)
                right_dir = True

                motor1_speed = motor2_speed
                left_dir = False

            if angular_velocity < 0:
                motor1_speed = max(75, motor1_speed)
                left_dir = True

                motor2_speed = motor1_speed
                right_dir = False

        with open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_input1.txt", "w") as f1, open("/home/ubuntu/ranger_object_finder/ranger_nav/motor/motor_input2.txt", "w") as f2:

            f1.write(f"{left_dir} {motor1_speed}")
            f2.write(f"{right_dir} {motor2_speed}")

        

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
