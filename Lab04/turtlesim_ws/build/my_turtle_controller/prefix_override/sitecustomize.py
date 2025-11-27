import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juan/ros2_ws/turtlesim_ws/install/my_turtle_controller'
