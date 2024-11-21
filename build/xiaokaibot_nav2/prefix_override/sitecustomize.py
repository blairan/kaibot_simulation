import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ctk/kaibot_ws/install/xiaokaibot_nav2'
