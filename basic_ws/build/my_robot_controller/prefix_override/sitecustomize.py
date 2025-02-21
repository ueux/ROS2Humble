import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ueux/Desktop/ROS2Humble/basic_ws/install/my_robot_controller'
