#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import  Twist
from turtlesim.msg import Pose
from turtlesim.srv import  SetPen
from functools import partial

class PoseSubNode(Node):
    def __init__(self):
        super().__init__("Turtle_controller")
        self.prev_x=0
        self.pose_subscriber_=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10) # Type , Name , Callback function , Queue size
        self.cmd_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10) # message type , topic name , queue size
        self.get_logger().info("Turtle Controller is started")

    def pose_callback(self,pose:Pose): #msg of type Pose
        cmd=Twist()
        if pose.x>9.0 or pose.x<2.0 or pose.y>9.0  or pose.y<2.0:
            cmd.linear.x=1.0
            cmd.angular.z=0.7
        else:
            cmd.linear.x=5.0
            cmd.angular.z=0.0
        self.cmd_vel_pub_.publish(cmd)

        #handel service call - how frequently we need service

        if pose.x>5.5 and self.prev_x<=5.5:
            self.prev_x=pose.x
            self.get_logger().info("Set color to green")
            self.call_set_pen_service(0,255,0,4,0)
        elif pose.x<=5.5 and self.prev_x>5.5:
            self.prev_x=pose.x
            self.get_logger().info("Set color to blue")
            self.call_set_pen_service(0,0,255,4,0)

    # almost same for services
    def call_set_pen_service(self,r,g,b,width,off):
        client=self.create_client(SetPen,"/turtle1/set_pen") # type , name
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service ...")
        request=SetPen.Request()
        request.r=r
        request.g=g
        request.b=b
        request.width=width
        request.off=off
        #future object
        future=client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    
    def callback_set_pen(self,future):
        try:
            response=future.result()
        except Exception as e:
            self.get_logger().error("Service call failed : %r"%(e,))


def main(args=None):
    rclpy.init(args=args)
    node=PoseSubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()