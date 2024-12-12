#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time


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

        # Transform broadcaster for odom to base_footprint
        self.odom_broadcaster = TransformBroadcaster(self)

        # Predefined velocity patterns
        self.velocities = {
            'stop': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'straight_forward': [-150.0, 0, 150.0, 0, -150.0, 0.0, 150.0, 0.0],
            'straight_reverse': [15.0, -1.0, -15.0, 1.0, 15.0, -1.0, -15.0, 1.0],
            'retract': [15.0, -10.0, -15.0, 10.0, 15.0, -10.0, -15.0, 15.0],
            'pushdown': [-0.0, 20.0, 0.0, -20.0, 0.0, 20.0, 0.0, -20.0],
            'left_forward': [-150.0, 0.0, 0.0, 0.0, -150.0, 0.0, 0.0, 0.0],
            'left_reverse': [15.0, -1.0, 0.0, 0.0, -15.0, 1.0, 0.0, 0.0],
            'left_retract': [15.0, -10.0, 0, 0, 15.0, -10.0, 0, 0],
            'left_pushdown': [-0.0, 20.0, 0.0, 0, 0.0, 20.0, 0.0, 0],
            'right_forward': [0, 0, 150, 0, 0, 0, -150.0, 0],
            'right_reverse': [0, 0, -15, 1.0, 0, 0, -15.0, 1.0],
            'right_retract': [0, 0, -15.0, 10.0, 0, 0, -15.0, 15.0],
            'right_pushdown': [-0.0, 0, 0.0, -20.0, 0.0, 0, 0.0, -20.0],
        }

        # Current direction
        self.direction = 'stop'

        # State toggle for flipping effect
        self.toggle_state = True
        self.flag = 0

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Timer to publish velocities and compute odometry
        self.timer = self.create_timer(0.5, self.update)  # Publish at 2 Hz
        self.get_logger().info('Velocity Publisher Node has started.')

    def cmd_vel_callback(self, msg):
        """Callback to handle /cmd_vel messages."""
        linear = msg.linear.x
        angular = msg.angular.z

        # Map linear and angular velocities to predefined directions
        if linear > 0:
            self.direction = 'straight_forward'
        elif linear < 0:
            self.direction = 'straight_reverse'
        elif angular > 0:
            self.direction = 'left_forward'
        elif angular < 0:
            self.direction = 'right_forward'
        else:
            self.direction = 'stop'

        self.get_logger().info(f'Direction changed to: {self.direction}')

    def update(self):
        """Publish velocities and compute odometry."""
        # Publish velocityright_
        velocity_command = Float64MultiArray()

        if self.direction in ['straight_forward', 'straight_reverse']:
            if self.toggle_state :
                velocity_command.data = self.velocities['straight_forward']
                self.flag = 1
                self.toggle_state = not self.toggle_state
            elif not self.toggle_state and self.flag == 1:
                velocity_command.data = self.velocities['pushdown']
                self.flag = 2
            elif not self.toggle_state and self.flag == 2:
                velocity_command.data = self.velocities['straight_reverse']
                self.flag = 0
            elif not self.toggle_state and self.flag == 0: 
                velocity_command.data = self.velocities['retract']
                self.toggle_state = not self.toggle_state
        elif self.direction in ['left_forward', 'left_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['left_forward']
                self.flag = 1
                self.toggle_state = not self.toggle_state
            elif not self.toggle_state and self.flag == 1:
                velocity_command.data = self.velocities['left_pushdown']
                self.flag = 2
            elif not self.toggle_state and self.flag == 2:
                velocity_command.data = self.velocities['left_reverse']
                self.flag = 0
            elif not self.toggle_state and self.flag == 0:
                velocity_command.data = self.velocities['left_retract']
                self.toggle_state = not self.toggle_state
        elif self.direction in ['right_forward', 'right_reverse']:
            if self.toggle_state:
                velocity_command.data = self.velocities['right_forward']
                self.flag = 1
                self.toggle_state = not self.toggle_state
            elif not self.toggle_state and self.flag == 1:
                velocity_command.data = self.velocities['right_pushdown']
                self.flag = 2
            elif not self.toggle_state and self.flag == 2:
                velocity_command.data = self.velocities['right_reverse']
                self.flag = 0
            elif not self.toggle_state and self.flag == 0:
                velocity_command.data = self.velocities['right_retract']
                self.toggle_state = not self.toggle_state
        else:
            velocity_command.data = self.velocities['stop']

        
        self.publisher.publish(velocity_command)
        time.sleep(1)

        # # Compute odometry
        # current_time = self.get_clock().now()
        # dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        # linear_velocity = 0.5 if self.direction.startswith('straight') else 0.0
        # angular_velocity = 0.1 if self.direction.startswith('left') else -0.1 if self.direction.startswith('right') else 0.0

        # delta_x = linear_velocity * math.cos(self.theta) * dt
        # delta_y = linear_velocity * math.sin(self.theta) * dt
        # delta_theta = angular_velocity * dt

        # self.x += delta_x
        # self.y += delta_y
        # self.theta += delta_theta

        # # Publish odometry
        # odom = Odometry()
        # odom.header.stamp = current_time.to_msg()
        # odom.header.frame_id = 'odom'
        # odom.child_frame_id = 'base_footprint'
        # odom.pose.pose.position.x = self.x
        # odom.pose.pose.position.y = self.y
        # odom.pose.pose.position.z = 0.0
        # odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        # odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        # odom.twist.twist.linear.x = linear_velocity
        # odom.twist.twist.angular.z = angular_velocity

        # self.odom_publisher.publish(odom)

        # # Broadcast transform
        # transform = TransformStamped()
        # transform.header.stamp = current_time.to_msg()
        # transform.header.frame_id = 'odom'
        # transform.child_frame_id = 'base_footprint'
        # transform.transform.translation.x = self.x
        # transform.transform.translation.y = self.y
        # transform.transform.translation.z = 0.0
        # transform.transform.rotation.z = math.sin(self.theta / 2.0)
        # transform.transform.rotation.w = math.cos(self.theta / 2.0)

        # self.odom_broadcaster.sendTransform(transform)
        # self.last_time = current_time


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
