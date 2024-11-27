#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        # Publisher for the /joint_vel/commands topic
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_vel/commands', 10)

        # Set the update rate (hz) - Reduced to 2 Hz for smoother movement
        self.update_rate = 2  # 2 Hz
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_velocity)

        # To keep track of the toggle state
        self.toggle_state = True  # Start with the first velocity pattern
        self.get_logger().info('Velocity Publisher Node has started.')

    def publish_velocity(self):
        # Define the velocity commands for each joint (4 joints)
        velocity_command = Float64MultiArray()

        if self.toggle_state:
            # First pattern: Move the front and back flippers in opposite directions
            # Front_left2_joint and Back_left2_joint go in opposite directions, Front_right2_joint and Back_right2_joint also go oppositely
            velocity_command.data = [3.0, 3.0, 3.0, 3.0]  # Front and back left joints forward, front and back right joints backward
        else:
            # Second pattern: Reverse the directions for a flipping effect
            velocity_command.data = [-3.0, -3.0, -3.0, -3.0]  # Front and back left joints backward, front and back right joints forward

        # Log the velocity being published for debugging
        self.get_logger().info(f'Publishing velocity command: {velocity_command.data}')

        # Publish the velocity command
        self.publisher.publish(velocity_command)

        # Toggle the state for the next cycle
        self.toggle_state = not self.toggle_state


def main(args=None):
    rclpy.init(args=args)
    
    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up on shutdown
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
