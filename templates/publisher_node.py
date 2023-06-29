#! /usr/bin/env python3
#Author: Samantha Smith, smith.15485@osu.edu

import rclpy
from rclpy.node import Node
from type_location import Type
import RelevantFunctions

class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.pub = self.create_publisher(Type, "service_node_name", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Type()
        msg.input = "input"
        msg.output = RelevantFunctions.function(msg.input)
        self.pub.publish(msg)
        self.get_logger().info("Publishing: '%s'" % msg.output)
    
def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
