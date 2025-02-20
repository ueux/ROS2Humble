#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("First_node")
        self.get_logger().info("Hello from Ros2")
        self.counter_=0
        self.create_timer(1.0,self.timer_callback)
    
    #callback and timer
    def timer_callback(self):
        self.get_logger().info(f"Hello {self.counter_}")
        self.counter_+=1


def main(args=None):
    rclpy.init(args=args)
    node=MyNode()

    #to keep node running
    rclpy.spin(node)

    rclpy.shutdown()


if __name__=='__main__':
    main()