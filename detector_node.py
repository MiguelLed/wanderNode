#!/usr/bin/env python
"""
Process scan messages to detect intruders.

Subscribes to: /scan
Publishes to:  /intruder

Author: Nathan Sprague && 
Version: 

"""
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan

import numpy as np
import math

class DetectorNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('intruder')

        self.prev_scan = None # Stores recently received scan messages.
        self.scan = None 

        self.intruder_pub = self.create_publisher(PointStamped, 'intruder', 10)
        self.create_timer(.1, self.timer_callback)

        self.create_subscription(LaserScan,'scan',
                                 self.scan_callback,
                                 qos_profile_sensor_data)

    def scan_callback(self, scan_msg):
        """Store the LaserScan msg."""
        self.prev_scan = self.scan
        self.scan = scan_msg

    def timer_callback(self):
        """Periodically check for intruders"""
        
    
        point = PointStamped()
        point.header.frame_id = "base_link"
	point.point.x = 1.0

        if self.prev_scan is not None:
            # TODO: PROCESS THE SCANS
            # Convert to numpy array
            self.prev_ranges = np.array(self.prev_scan.ranges)
            self.curr_ranges = np.array(self.scan.ranges)

            # Clean the arrays
            self.prev_ranges[np.isinf(self.prev_ranges)] = 0
            self.prev_ranges[np.isnan(self.prev_ranges)] = 0
            self.curr_ranges[np.isinf(self.curr_ranges)] = 0
            self.curr_ranges[np.isnan(self.curr_ranges)] = 0

            self.proc_ranges = self.curr_ranges - self.prev_ranges
            self.proc_ranges = np.abs(self.proc_ranges)

            if(np.max(self.proc_ranges) > 0.1) and (np.max(self.proc_ranges) < 1.5):
                self.max_value = np.max(self.proc_ranges)
                self.max_index = np.argmax(self.proc_ranges)
                self.length_to_intruder = self.curr_ranges[self.max_index]

                if(self.length_to_intruder > 0): 
                    
                    # print()
                    # print("Biggest change:", self.max_value)
                    # print("Intruder index:", self.max_index)
                    # print("Length to intruder:", self.length_to_intruder)
                    # print("-------------------")

                    # self.max_index = 360 - self.max_index
			
			#look at scan message
			#this is reading the index of the max value
			#could be reading at an insane degree
                    point.point.x = self.length_to_intruder * math.sin(self.max_index)
                    point.point.y = self.length_to_intruder * math.cos(self.max_index)
			#use log instead of print
                    print()
                    print("Intruder X: ", str(point.point.x))
                    print("Intruder Y: ", str(point.point.y))

        self.intruder_pub.publish(point)

def main():
    rclpy.init()
    detector_node = DetectorNode()
    rclpy.spin(detector_node)

    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
