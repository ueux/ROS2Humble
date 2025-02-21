#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubNode(Node):
    def __init__(self):
        super().__init__("Pose_subscriber")
        self.pose_subscriber_=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10) # Type , Name , Callback function , Queue size

    def pose_callback(self,msg:Pose): #msg of type Pose
        self.get_logger().info("("+str(msg.x)+","+str(msg.y)+")")

        

def main(args=None):
    rclpy.init(args=args)
    node=PoseSubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()