import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zheng-yang/robotik-homework/install/assignment2_publisher_subscriber_avd'
