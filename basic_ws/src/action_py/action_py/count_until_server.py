#!/usr/bin/env python3
import time
import rclpy 
from rclpy.node import Node
from rclpy.action import ActionServer 
from rclpy.action.server import ServerGoalHandle

from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_=ActionServer(self,CountUntil,"count_until",execute_callback=self.execute_cb)
        self.get_logger().info("Count until server has been started")
    def execute_cb(self,goal_handle:ServerGoalHandle):
        self.get_logger().info("Executing goal...")
        target=goal_handle.request.target_number
        period = goal_handle.request.wait_duration
        counter=0
        for i in range(target):
            counter+=1
            self.get_logger().info(f"{counter}")
            time.sleep(period)
        #Once done, set goal final state
        goal_handle.succeed()
        #send result
        result=CountUntil.Result()
        result.reached_number=counter
        return result
def main(args=None):
    rclpy.init(args=args)
    node=CountUntilServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()