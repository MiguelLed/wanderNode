#!/usr/bin/env python

""" Ryan Lewis, Miguel Ledesma, Ryan Maring """

import rclpy
import rclpy.node

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WandererNode(rclpy.node.Node):

    
    def __init__(self):
        super().__init__('wanderer')
        
        self.laser_range = None

        self.move_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.1, self.timer_callback)

        self.create_subscription(LaserScan, 'scan',
            self.location_callback, qos_profile_sensor_data)

    def location_callback(self, laser_msg):
        self.laser_range = laser_msg
        print(self.laser_range.ranges[0])

    def timer_callback(self):

        if self.laser_range is not None:
            twist = Twist()
            if self.laser_range.ranges[0] < 1:
                twist.angular.z = 1.0
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            self.move_publisher.publish(twist)

def main():


    rclpy.init()
    wanderer_node = WandererNode()
    rclpy.spin(wanderer_node)
    wanderer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
