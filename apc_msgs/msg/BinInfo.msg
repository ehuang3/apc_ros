# BInfo
geometry_msgs/Pose bin_pose  # Camera Frame (We do not need shelf pose given this)
# Unless it is easier for Eric to give shelf pose and robot orientation
geometry_msgs/Vector3 bin_size  # Assume symmeteric, in meters, x, y, z extent

std_msgs/String bin_name
apc_msgs/Object[] object_list