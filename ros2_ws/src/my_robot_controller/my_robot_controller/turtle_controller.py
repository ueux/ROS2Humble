#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import  Twist
from turtlesim.msg import Pose

class PoseSubNode(Node):
    def __init__(self):
        super().__init__("Turtle_controller")
        self.pose_subscriber_=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10) # Type , Name , Callback function , Queue size
        self.cmd_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10) # message type , topic name , queue size
        self.get_logger().info("Turtle Controller is started")

    def pose_callback(self,pose:Pose): #msg of type Pose
        cmd=Twist()
        if pose.x>9.0 or pose.x<2.0 or pose.y>9.0  or pose.y<2.0:
            cmd.linear.x=1.0
            cmd.angular.z=0.9
        else:
            cmd.linear.x=5.0
            cmd.angular.z=0.0
        
        self.cmd_vel_pub_.publish(cmd)

        

def main(args=None):
    rclpy.init(args=args)
    node=PoseSubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()