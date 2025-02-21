#!/usr/bin/env python3
import time
import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient 

from my_robot_interfaces.action import CountUntil

class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server=ActionClient(self,CountUntil,"count_until")
        self.get_logger().info("Count until server has been started")
    def send_goal(self,target,period):
        #Wait for the server to be up
        self.count_until_server.wait_for_server()
        #create goal
        goal=CountUntil.Goal()
        goal.target_number=target
        goal.wait_duration=period
        self.get_logger().info(f"Sending goal request: {goal}")
        self.send_goal_future=self.count_until_server.send_goal_async(goal,feedback_callback=self.feedback_cb)
def main(args=None):
    rclpy.init(args=args)
    node=CountUntilClientNode()
    node.send_goal(5,1)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()