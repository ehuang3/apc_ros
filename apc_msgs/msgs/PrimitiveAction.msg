# An primitive manipulation action

# The maximum amount of time the motion planner is allowed to plan for
float64 allowed_planning_time

# Planning options
PlanningOptions planning_options

# which group to be used to plan
string group_name

# the name that the attached object to move
string object_name

# if the action should be executed relative to the object
bool relative_to_object

# if the action should be executed relative to the previous eef pose
bool relative_to_previous_pose

# if applicable, the trajectory of the joints
trajectory_msgs/JointTrajectory joint_trajectory

# if applicable, the trajectory of the end-effector
geometry_msgs/PoseArray eef_trajectory

# the trajectory of the manipulated object
geometry_msgs/PoseArray object_trajectory

# the recorded force-torque profile
geometry_msgs/Pose[] ft_profile

# the recorded tactile-sensor profile
schunk_sdh/TactileSensor[] ts_profile



# a list of possible transformations for placing the object
PlaceLocation[] place_locations

# if the user prefers setting the eef pose (same as in pick) rather than 
# the location of an end effector, this flag should be set to true
bool place_eef

# the name that the support surface (e.g. table) has in the collision world
# can be left empty if no name is available
string support_surface_name

# whether collisions between the gripper and the support surface should be acceptable
# during move from pre-place to place and during retreat. Collisions when moving to the
# pre-place location are still not allowed even if this is set to true.
bool allow_gripper_support_collision

# Optional constraints to be imposed on every point in the motion plan
Constraints path_constraints

# The name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id

# an optional list of obstacles that we have semantic information about
# and that can be touched/pushed/moved in the course of grasping;
# CAREFUL: If the object name 'all' is used, collisions with all objects are disabled during the approach & lift.
string[] allowed_touch_objects



