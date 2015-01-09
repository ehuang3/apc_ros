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

# if true, record a dense trajectory from teleoperation for playback
bool record_dense_trajectory

# if true, linearly interpolate in the trajectory space
bool interpolate_linear

# the type of trajectory message to use
int32 use_type_trajectory
int32 USE_JOINT_TRAJECTORY = 0
int32 USE_EEF_TRAJECTORY = 1

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
bool use_haptic_profiles

# the recorded force-torque profile
geometry_msgs/Pose[] ft_profile

# the recorded tactile-sensor profile
TactileSensor[] ts_profile

# duration of trajectory execution
float64 duration
