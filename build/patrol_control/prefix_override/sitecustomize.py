import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pes2ug23cs024/MAR_NIGHT_PATROL_ROBOT/install/patrol_control'
