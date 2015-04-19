# a primitive manipulation action

# Header information
Header header

# the maximum amount of time the motion planner is allowed to plan for
float64 allowed_planning_time

# the name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id

######################################################################
# Action
######################################################################

# unique id for this action
string unique_id

# the name of this primitive action
string action_name

######################################################################
# Robot
######################################################################

# which group to be used to plan
string group_id

######################################################################
# Object
######################################################################

# The name of the attached object to move.
string object_id

# The unique key associated with this object in the planning world.
string object_key

# The name of the link the object is attached to.
string attached_link_id

# The trajectory of the manipulated object relative to link frame.
geometry_msgs/PoseArray object_trajectory

######################################################################
# End-Effector Trajectory
######################################################################

# The ID of the frame the trajectory is executed relative to.
string frame_id

# The unique associated with this frame in the planning world.
string frame_key

# The name of the end-effector link.
string eef_link_id

# The cartesian trajectory of the end-effector link in the given
# frame.
geometry_msgs/PoseArray eef_trajectory

# Convenience variable to tell whether the start and goal states are
# locked together.
bool eef_locked

######################################################################
# Joint Trajectory
######################################################################

# if applicable, the trajectory of the joints
trajectory_msgs/JointTrajectory joint_trajectory

######################################################################
# Options
######################################################################

# if true, interpolate in cartesian space
bool interpolate_cartesian

# stop trajectory execution on contact
bool monitor_contact

# use previous haptic experiences to gauge correctness of action execution
bool monitor_haptic_profile

# the name that the support surface (e.g. table) has in the collision world
# can be left empty if no name is available
string support_surface_name

# if the action should be executed relative to the previous eef pose
bool relative_to_previous_pose

# if true, a dense trajectory from teleoperation for playback
bool dense_trajectory

# the type of trajectory message to use
bool use_eef_trajectory

# the recorded force-torque profile
geometry_msgs/Pose[] ft_profile

# the recorded tactile-sensor profile
TactileSensor[] ts_profile

# duration of trajectory execution
float64 duration
