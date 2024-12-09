#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import math


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        # Publisher for the /joint_vel/commands topic
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_vel/commands', 10)

        # Subscriber for the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for the odometry topic
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Publisher for the LaserScan topic (optional for testing)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Predefined velocity patterns
        self.base_velocities = {
            'stop': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'straight_forward': [1.0, 1.0, 1.0, 1.0, 0.8, 0.0, 0.8, 0.0],
        }

        self.speed_profiles = {'normal': 1.0}
        self.current_speed_profile = 'normal'
        self.direction = 'stop'

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_time = self.get_clock().now()

        # Frame names
        self.odom_frame = "odom"
        self.base_frame = "base_link"
        self.sensor_frame = "sensor_link"

        # Timer to update velocities, odometry, and transforms
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        self.get_logger().info('Velocity Publisher Node has started.')

    def cmd_vel_callback(self, msg):
        """Callback to handle /cmd_vel messages."""
        linear = msg.linear.x
        angular = msg.angular.z

        self.direction = 'straight_forward' if linear > 0 else 'stop'
        self.linear_velocity = linear
        self.angular_velocity = angular

    def update(self):
        """Update velocities, odometry, and broadcast transforms."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Compute odometry
        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        delta_theta = self.angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish odometry
        current_time_msg = current_time.to_msg()
        odom = Odometry()
        odom.header.stamp = current_time_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.odom_publisher.publish(odom)

        # Broadcast transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = current_time_msg
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(transform)

        # Broadcast transform from base_link to sensor_link
        sensor_transform = TransformStamped()
        sensor_transform.header.stamp = current_time_msg
        sensor_transform.header.frame_id = self.base_frame
        sensor_transform.child_frame_id = self.sensor_frame
        sensor_transform.transform.translation.x = 0.2
        sensor_transform.transform.translation.y = 0.0
        sensor_transform.transform.translation.z = 0.3
        sensor_transform.transform.rotation.x = 0.0
        sensor_transform.transform.rotation.y = 0.0
        sensor_transform.transform.rotation.z = 0.0
        sensor_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(sensor_transform)

        # Publish a dummy LaserScan message (optional)
        scan = LaserScan()
        scan.header.stamp = current_time_msg
        scan.header.frame_id = self.sensor_frame
        scan.angle_min = -1.57  # -90 degrees
        scan.angle_max = 1.57  # 90 degrees
        scan.angle_increment = 0.01
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.2
        scan.range_max = 5.0
        scan.ranges = [1.0 + 0.1 * math.sin(i * 0.1) for i in range(314)]  # Example scan data
        self.scan_publisher.publish(scan)

        # Publish predefined velocity pattern
        velocity_command = Float64MultiArray()
        scale = self.speed_profiles[self.current_speed_profile]
        velocity_command.data = [v * scale for v in self.base_velocities[self.direction]]
        self.publisher.publish(velocity_command)

        # Update the last time
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
