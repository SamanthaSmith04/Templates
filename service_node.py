#! /usr/bin/env python3
#Author: Samantha Smith, smith.15485@osu.edu

import rclpy
from rclpy.node import Node
from service_location import ServiceMessage
import RelevantFunctions

class ServiceNode(Node):
    def __init__(self):
        super().__init__("service_node")
        self.srv = self.create_service(ServiceMessage, "service_node_name", self.srv_call)

    def srv_call(self, request, response):
        response.output = RelevantFunctions.function(request.input)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()