#! /usr/bin/env python3
#Author: Samantha Smith, smith.15485@osu.edu

import rclpy
from rclpy.node import Node
from service_location import ServiceMessage

class ClientNode(Node):
    def __init__(self):
        super().__init__("client_node")
        self.cli = self.create_client(ServiceMessage, "service_node_name")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        self.req = ServiceMessage.Request()

    def send_request(self):
        self.req.input = "input"
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback)

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()