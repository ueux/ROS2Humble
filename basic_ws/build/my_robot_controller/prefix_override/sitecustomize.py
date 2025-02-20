import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vboxuser/Desktop/ROS2Humble/ros2_ws/install/my_robot_controller'
