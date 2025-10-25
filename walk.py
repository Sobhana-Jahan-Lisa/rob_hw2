#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random

class Walk(Node):
    def __init__(self):
        super().__init__('walk')
        self.subscription = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.sensor_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 0.5  # minimum distance before reacting
        self.stuck_counter = 0
        self.get_logger().info('Robot started moving....')

    def sensor_callback(self, msg):
        data = np.array(msg.ranges)
        data = np.where(np.isinf(data), 5.0, data)  # Replace inf with max sensor range

        # Split into regions
        front = np.min(data[len(data)//3: 2*len(data)//3])
        left = np.min(data[2*len(data)//3:])
        right = np.min(data[:len(data)//3])

        twist = Twist()

        # If front is blocked
        if front < self.safe_distance:
            if left > right:
                twist.angular.z = 0.5   # turn left
            else:
                twist.angular.z = -0.5  # turn right
            twist.linear.x = 0.0
        else:
            # Free ahead
            twist.linear.x = 2.2
            twist.angular.z = 0.0

        # If robot is stuck (front, left, right all blocked)
        if front < self.safe_distance and left < self.safe_distance and right < self.safe_distance:
            self.stuck_counter += 1
            twist.linear.x = -0.3   # back up
            twist.angular.z = random.uniform(-0.5, 0.5)   # and turn a bit
        else:
            self.stuck_counter = 0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    walk = Walk()

    try:
        rclpy.spin(walk)
    except KeyboardInterrupt:
        walk.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        walk.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
