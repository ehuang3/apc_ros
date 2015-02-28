# Motion plan request

# Starting world state.
moveit_msgs/PlanningScene scene

# Starting state updates. If certain joints should be considered
# at positions other than the current ones, these positions should
# be set here
moveit_msgs/RobotState start_state

# The goal states to plan for.
Constraints[] goal_constraints
