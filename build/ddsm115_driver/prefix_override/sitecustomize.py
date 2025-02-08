import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/circuitcoder1101/ros2_agv/install/ddsm115_driver'
