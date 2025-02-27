import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from math import sin, cos
from nav_msgs.msg import Odometry

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Create a TF broadcaster to send the transform
        self.broadcaster = TransformBroadcaster(self)

        # Create an odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Create a timer to publish at a fixed rate (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds

        # Placeholder for motor updates (replace with actual motor data)
        # You should calculate the robot's movement here based on the I2C motor control
        linear_velocity = 0.0 #* motor_speed / 100  m/s (example value)
        angular_velocity = 0.0  # rad/s (example value)

        # Update position and orientation based on velocity
        self.x += linear_velocity * dt * cos(self.theta)
        self.y += linear_velocity * dt * sin(self.theta)
        self.theta += angular_velocity * dt

        # Convert angle to quaternion for the TF message
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)

        # Publish the TF transform from odom to base_link
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = current_time.to_msg()
        t_odom_base.header.frame_id = "odom"
        t_odom_base.child_frame_id = "base_link"
        t_odom_base.transform.translation.x = self.x
        t_odom_base.transform.translation.y = self.y
        t_odom_base.transform.translation.z = 0.0  # Assuming 2D motion
        t_odom_base.transform.rotation.x = qx
        t_odom_base.transform.rotation.y = qy
        t_odom_base.transform.rotation.z = qz
        t_odom_base.transform.rotation.w = qw

        # Broadcast the odom -> base_link transform
        self.broadcaster.sendTransform(t_odom_base)

        # Create and publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0  # Assuming 2D motion
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

        # Now broadcast the TF from base_link to laser (this is the new part)
        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = current_time.to_msg()
        t_base_laser.header.frame_id = "base_link"
        t_base_laser.child_frame_id = "laser"
        
        # Assuming the laser sensor is positioned 1 meter ahead of the robot in the x direction
        t_base_laser.transform.translation.x = 1.0  # 1 meter ahead of the robot
        t_base_laser.transform.translation.y = 0.0  # Same height
        t_base_laser.transform.translation.z = 0.5  # Height of laser from ground (adjust as needed)

        # Assuming the laser is aligned with the robot (no rotation in this example)
        t_base_laser.transform.rotation.x = 0.0
        t_base_laser.transform.rotation.y = 0.0
        t_base_laser.transform.rotation.z = 0.0
        t_base_laser.transform.rotation.w = 1.0  # No rotation

        # Broadcast the base_link -> laser transform
        self.broadcaster.sendTransform(t_base_laser)

        # Save current time for the next iteration
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
