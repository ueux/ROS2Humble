#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("Draw_cycle")
        self.cmd_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10) # message type , topic name , queue size
        self.timer_=self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info("Draw cycle node is started")
    def send_velocity_command(self):
        msg=Twist()
        msg.linear.x=2.0
        msg.angular.z=1.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()