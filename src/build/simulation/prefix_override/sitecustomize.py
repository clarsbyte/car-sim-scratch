import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/clarissa/Documents/pers-robotics/car-scratch/src/install/simulation'
