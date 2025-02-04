import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/afs/inf.ed.ac.uk/user/s22/s2281597/ranger_object_finder/ranger_nav/ros_ws/install/ranger_nav'
