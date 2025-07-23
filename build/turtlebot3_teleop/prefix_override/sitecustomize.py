import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rodrigo/Downloads/ROS2Gazebo/install/turtlebot3_teleop'
