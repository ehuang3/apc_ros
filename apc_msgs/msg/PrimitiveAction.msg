# a primitive manipulation action

# Header information
Header header

# unique id for this action
int64 unique_id

# the name of this primitive action
string action_name

# the maximum amount of time the motion planner is allowed to plan for
float64 allowed_planning_time

# planning options
moveit_msgs/PlanningOptions planning_options

# the name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id

# which group to be used to plan
string group_name

# the name that the attached object to move
string object_name

# the name that the support surface (e.g. table) has in the collision world
# can be left empty if no name is available
string support_surface_name

# if the action should be executed relative to the object
bool relative_to_object

# if the action should be executed relative to the previous eef pose
bool relative_to_previous_pose

# if true, a dense trajectory from teleoperation for playback
bool dense_trajectory

# if true, interpolate in cartesian space
bool interpolate_cartesian

# the type of trajectory message to use
bool use_eef_trajectory

# if applicable, the trajectory of the joints
trajectory_msgs/JointTrajectory joint_trajectory

# if applicable, the trajectory of the end-effector
geometry_msgs/PoseArray eef_trajectory

# optional constraints to be imposed on every point in the motion plan
moveit_msgs/Constraints path_constraints

# the trajectory of the manipulated object
geometry_msgs/PoseArray object_trajectory

# stop trajectory execution on contact
bool stop_on_contact

# use previous haptic experiences to gauge correctness of action execution
bool use_haptic_profile

# the recorded force-torque profile
geometry_msgs/Pose[] ft_profile

# the recorded tactile-sensor profile
TactileSensor[] ts_profile

# duration of trajectory execution
float64 duration
