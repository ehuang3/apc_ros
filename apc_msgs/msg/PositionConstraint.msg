# This message contains the definition of a position constraint.

Header header

# The rigid body this constraint refers to
string body_name

# The offset for the target point on the body we are planning for 
geometry_msgs/Vector3 target_point

# optional axis error tolerances specified
float64 absolute_x_axis_tolerance
float64 absolute_y_axis_tolerance
float64 absolute_z_axis_tolerance

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight
