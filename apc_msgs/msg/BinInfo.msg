# BInfo
std_msgs/Header header

# Pose of shelf in the header frame. Usually crichton origin.
geometry_msgs/Pose pose_shelf_frame

# Pose of bin in shelf.
geometry_msgs/Pose pose_bin_shelf

# geometry_msgs/Pose bin_pose  # Camera Frame (We do not need shelf pose given this)

# Unless it is easier for Eric to give shelf pose and robot orientation
geometry_msgs/Vector3 bin_size  # Assume symmeteric, in meters, x, y, z extent

string bin_name
string target_item
apc_msgs/Object[] object_list
