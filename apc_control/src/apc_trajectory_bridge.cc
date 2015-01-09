#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <apc_msgs/FollowPrimitivePlanAction.h>
#include <Eigen/Dense>
#include <sns.h>
#include <apc_path/Path.h>
#include <apc_path/Trajectory.h>

typedef actionlib::SimpleActionServer<apc_msgs::FollowPrimitivePlanAction> ActionServer;
typedef apc_msgs::FollowPrimitivePlanGoalConstPtr GoalConstPtr;
typedef apc_msgs::FollowPrimitivePlanFeedback Feedback;
typedef apc_msgs::FollowPrimitivePlanResult Result;
typedef apc_msgs::PrimitiveAction Action;

struct Group
{
    std::map<std::string, int> map;      // Mapping from joint name to reference index.
    std::string name_state;              // Name of read motor states channel.
    std::string name_ref;                // Name of send reference commands channel.
    ach_channel_t chan_state;            // Channel to read motor states from.
    ach_channel_t chan_ref;              // Channel to send reference commands to.
    enum sns_motor_mode mode;            // Operating mode of motor reference commands.
};

struct Parameters
{
    std::string action_topic;            // Name of follow primitive plan action topic.
    bool allow_execution;                // Allow trajectory execution on the robot.

    double max_vel;                      // Maximum joint velocity along the path.
    double max_accel;                    // Maximum joint acceleration along the path.

    Group left_hand;                     // The left hand group.
    Group right_hand;                    // The right hand group.
    Group left_arm;                      // The left arm group.
    Group right_arm;                     // The right arm group.
    Group torso;                         // The torso group.

    // Read parameters off ROS node.
    bool getParams(ros::NodeHandle& node);

} tbridge_params;

// Open channel and setup signal handling.
bool openChannel(std::string chan_name, ach_channel_t* chan)
{
    sns_chan_open( chan, chan_name.c_str(), NULL );
    {
        ach_channel_t *chans[] = {chan, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }
    return true;
}

bool openChannels(Group* group)
{
    bool success = true;
    success &= openChannel(group->name_state, &group->chan_state);
    success &= openChannel(group->name_ref,   &group->chan_ref);
    return success;
}

bool openChannels(Parameters* params)
{
    bool success = true;
    success &= openChannels(&params->left_hand);
    success &= openChannels(&params->right_hand);
    success &= openChannels(&params->left_arm);
    success &= openChannels(&params->right_arm);
    success &= openChannels(&params->torso);
    return success;
}

Result check_joint_names(const std::vector<std::string>& j_n, const std::map<std::string, int>& j_m)
{
    bool valid = true;
    if (j_n.size() != j_m.size())
        valid = false;
    typedef std::map<std::string, int>::const_iterator MapIterator;
    for (MapIterator iter = j_m.begin(); iter != j_m.end(); ++iter)
        if (std::find(j_n.begin(), j_n.end(), iter->first) == j_n.end())
            valid = false;
    Result r;
    if (!valid)
    {
        r.error_code = Result::INVALID_JOINTS;
        r.error_string = "Mismatch between the expected controller joints and those provided in the goal";
    }
    return r;
}

Result get_time(struct timespec* time)
{
    Result r;
    // Get the current time.
    if( clock_gettime( ACH_DEFAULT_CLOCK, time ) )
    {
        SNS_LOG(LOG_ERR, "clock_gettime failed: %s\n", strerror(errno));
        r.error_code = Result::FAILED_CLOCK_GETTIME;
        r.error_string = "Failed clock_gettime";
    }
    return r;
}

Result read_motor_state(struct sns_msg_motor_state* msg,
                        struct timespec* time_out,
                        Group* group,
                        Parameters* params)
{
    Result r;

    // The frame size of the buffer.
    size_t frame_size;

    // Pointer to buffer that stores the message.
    void* buf;

    // We will get the latest message with timeout.
    int opt = ACH_O_LAST | (time_out ? ACH_O_WAIT : 0);

    // Get the motor state message.
    ach_status_t s = sns_msg_local_get( &group->chan_state, &buf, &frame_size, time_out, opt );

    // Handle the return status. ach ok and missed frame are both
    // acceptable as we only want the latest motor state.
    switch (s)
    {
    case ACH_OK:
    case ACH_MISSED_FRAME:
    {
        // Sanity check the size of the message.
        if (frame_size != sns_msg_motor_state_size_n(group->map.size()))
        {
            r.error_code = Result::INVALID_JOINTS;
            r.error_string = "Failed to get the correct motor states";
            break;
        }

        // Convert raw buffer to motor state message.
        memcpy(msg, buf, frame_size);

        break;
    }
    case ACH_STALE_FRAMES:
    {
        r.error_code = Result::STALE_FRAMES;
        r.error_string = "No new data has been published to channel: " + group->name_state;

        break;
    }
    default:
        // Log error on failure.
        SNS_LOG(LOG_ERR, "Failed ach_get: %s\n", ach_result_to_string(s));

        r.error_code = Result::INVALID_JOINTS;
        r.error_string = std::string("Failed ach_get: ") + ach_result_to_string(s);

        break;
    }

    return r;
}

Result build_motor_ref(sns_msg_motor_ref* msg,
                       const Eigen::VectorXd& point,
                       Group* group,
                       Parameters* params)
{
    Result r;

    // Initialize the message.
    sns_msg_motor_ref_init( msg, point.size() );

    // Set motor control mode.
    msg->mode = group->mode;

    // Copy the velocity command to the motor reference message.
    AA_MEM_CPY(msg->u, point.data(), msg->header.n);

    // Set a timeout 1 second in the future.
    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_msg_set_time( &msg->header, &now, 1e9 ); /* 1 second duration */

    return r;
}

Result send_motor_ref(sns_msg_motor_ref* msg,
                      Group* group,
                      Parameters* params)
{
    Result r;

    // The ACH return status.
    ach_status_t status = ACH_OK;

    // Send reference command message, if enabled.
    if (params->allow_execution)
        status = ach_put( &group->chan_ref, msg, sns_msg_motor_ref_size(msg) );
    else
    {
        std::stringstream cmd;
        for (int i=0; i < msg->header.n; i++)
            cmd << msg->u[i] << " ";
        ROS_INFO_STREAM("sim " << group->name_ref << ": " << cmd.str());
    }

    // Handle ach errors.
    if(status != ACH_OK)
    {
        SNS_LOG(LOG_ERR, "ach_put failed: %s\n", ach_result_to_string(status));
        r.error_code = Result::INVALID_JOINTS;
        r.error_string = std::string("Failed ach_put: ") + ach_result_to_string(status);
    }

    return r;
}

Result set_and_check_start_state(const Action& action,
                                 Eigen::VectorXd& point,
                                 Group* group,
                                 Parameters* params)
{
    Result r;

    // If we are not executing, fake the robot state with the action's start state.
    if (!params->allow_execution)
    {
        const std::vector<std::string>& joint_names = action.joint_trajectory.joint_names;
        const trajectory_msgs::JointTrajectoryPoint& start = action.joint_trajectory.points.front();
        point.resize(joint_names.size());
        for (int i = 0; i < joint_names.size(); i++)
            point[i] = start.positions[group->map[joint_names[i]]];
        return r;
    }

    // Get the joint trajectory
    const trajectory_msgs::JointTrajectory& T = action.joint_trajectory;

    // Resize point to number of motors in group.
    point.resize(T.points[0].positions.size());

    // Locally allocate a motor state message.
    struct sns_msg_motor_state* msg = sns_msg_motor_state_local_alloc(point.size());

    // Read time.
    struct timespec ts;
    r = get_time(&ts);
    if (r.error_code)
        return r;

    // Apply a timeout when reading messages.
    const int64_t ms = 5;
    struct timespec to = sns_time_add_ns( ts, ms*1e6 );

    // Read motor state.
    r = read_motor_state(msg, &to, group, params);
    if (r.error_code)
        return r;

    // Set and check start state.
    for (int i = 0; i < T.joint_names.size(); i++)
    {
        // Get the joint index.
        int j_i = group->map[T.joint_names[i]];

        // If the current state delta and desired start position
        // differ by more than 1 degree, abort.
        double delta = std::abs(msg->X[j_i].pos - T.points[0].positions[i]);
        if (delta > 0.017453)   // 1 degree in radians
        {
            r.error_code = Result::PATH_TOLERANCE_VIOLATED;
            r.error_string = "Joint differs by more than 1 degree: " + T.joint_names[i];
            break;
        }

        // Otherwise, copy current state into point.
        point[j_i] = msg->X[j_i].pos;
    }

    // Print out errors.
    if (r.error_code)
        for (int i = 0; i < T.joint_names.size(); i++)
        {
            int j = group->map[T.joint_names[i]];
            ROS_ERROR("%s desired[%d]: %+.6f actual[%d]: %+.6f",
                      T.joint_names[i].c_str(),
                      i,
                      T.points[0].positions[i],
                      j,
                      msg->X[j].pos);
        }

    return r;
}

Result set_velocity_to_zero(const Action& action,
                            Group* group,
                            Parameters* params)
{
    Result r;
    ROS_INFO("Setting velocity to 0");
    if (group->mode == SNS_MOTOR_MODE_VEL)
    {
        // Set to zero.
        Eigen::VectorXd p(group->map.size());
        p.setZero();

        // Build reference command message.
        struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc(p.size());
        r = build_motor_ref(msg, p, group, params);
        if (r.error_code)
            return r;

        // Send motor reference command.
        r = send_motor_ref(msg, group, params);
        if (r.error_code)
            return r;

    }
    return r;
}

Result execute_trajectory(const Action& action,
                          ActionServer* server,
                          Group* group,
                          Parameters* params)
{
    // Return value.
    Result r;
    r.error_code = Result::SUCCESSFUL;

    ROS_INFO("Preparing trajectory: %s", action.group_name.c_str());

    // PRE-CONDITION: Assert joints in map and action agree.
    r = check_joint_names(action.joint_trajectory.joint_names, group->map);
    if (r.error_code)
        return r;

    // Get the number of degrees of freedom in this group.
    const int n_dof = group->map.size();

    // TODO Assert trajectory is not malformed.

    // TODO Force safety limits.

    // TODO Haptic profile monitoring.

    // TODO Check goal tolerances.

    // Set velocity to zero.
    // BUG There is a bug in SCHUNK firmware that requires this.
    // BUG We do this before reading the motor state because the
    // BUG joints tend to move slightly even after setting velocity
    // BUG to zero.
    {
        r = set_velocity_to_zero(action, group, params);
        if (r.error_code)
            return r;

        // Wait 10ms to let the joints stop moving.
        usleep( (useconds_t) 10 * 1e3 ); // Based on can402 control loop frequency
    }

    // PRE-CONDITION: Assert group state is near the start point.
    Eigen::VectorXd p;
    r = set_and_check_start_state(action, p, group, params);
    if (r.error_code)
        return r;

    // Convert trajectory into a path, a.k.a. list of eigen vectors.
    std::list<Eigen::VectorXd> P;
    {
        // Add start state.
        P.push_back(p);

        // Add the trajectory.
        const trajectory_msgs::JointTrajectory& T = action.joint_trajectory;
        for (int i = 0; i < T.points.size(); i++)
        {
            // Convert trajectory point to eigen vector.
            for (int j = 0; j < p.rows(); j++)
                p[group->map[T.joint_names[j]]] = T.points[i].positions[j];
            P.push_back(p);
        }
    }

    // Create velocity and acceleration limits.
    const Eigen::VectorXd max_vel   = params->max_vel * Eigen::VectorXd::Ones(n_dof);
    const Eigen::VectorXd max_accel = params->max_accel * Eigen::VectorXd::Ones(n_dof);

    // Pass path through Toby's code.
    Trajectory T(Path(P, 0.1), max_vel, max_accel);

    // Abort if the trajectory is not valid.
    if(!T.isValid())
    {
        r.error_code = Result::INVALID_GOAL;
        r.error_string = "Failed to optimize trajectory";
        return r;
    }

    // Get the total trajectory time.
    double duration = T.getDuration();

    ROS_INFO("Executing trajectory: %s", action.group_name.c_str());

    // Current timespec.
    struct timespec ts;
    r = get_time( &ts );
    if (r.error_code)
        return r;

    // Record the start time.
    double start = ts.tv_sec + ts.tv_nsec / (double) 1e9;

    // The current time in seconds.
    double time = start;

    // Execute trajectory on robot.
    while (time < start + duration)
    {
        // Check for preemption.
        if (server->isPreemptRequested())
        {
            ROS_INFO("Trajectory execution preempted");
            server->setPreempted();
            r.error_code = Result::PREEMPTED;
            r.error_string = "Trajectory execution preempted";
            break;
        }

        // Get the current time.
        r = get_time( &ts );

        // On failure, break to ensure post-conditions.
        if (r.error_code) break;

        // Convert the current time to seconds.
        time = ts.tv_sec + ts.tv_nsec / (double) 1e9;

        // Get the reference command at this time.
        Eigen::VectorXd cmd;
        switch (group->mode)
        {
        case SNS_MOTOR_MODE_POS:
            cmd = T.getPosition(time - start);
            break;
        case SNS_MOTOR_MODE_VEL:
            cmd = T.getVelocity(time - start);
            break;
        default:
            r.error_code = Result::INVALID_GOAL;
            std::stringstream err;
            err << "Unsupported motor mode: " << group->mode;
            r.error_string = err.str();
            ROS_ERROR_STREAM(err.str());
            break;
        }

        // On failure, break to ensure post-conditions.
        if (r.error_code) break;

        // Locally allocate a reference command message.
        struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc(p.size());

        // Build reference command message.
        r = build_motor_ref(msg, cmd, group, params);

        // On failure, break to ensure post-conditions.
        if (r.error_code) break;

        // Send motor reference command.
        r = send_motor_ref(msg, group, params);

        // On failure, break to ensure post-conditions.
        if (r.error_code) break;

        // Clear local memory.
        aa_mem_region_local_release();

        // Sleep for 10 ms.
        usleep( (useconds_t) 10 * 1e3 ); // Based on can402 control loop frequency
    }

    // POST-CONDITION: Set velocities to zero.
    set_velocity_to_zero(action, group, params);

    // POST-CONDITION: Local memory is cleared.
    aa_mem_region_local_release();

    return r;
}

void execute(const GoalConstPtr& goal, ActionServer* action_server, Parameters* params)
{
    // The action request result.
    Result r;

    ROS_INFO("Received %lu actions to execute", goal->plan.actions.size());

    // For each primitive action in the primitive plan...
    for (int i = 0; i < goal->plan.actions.size(); i++)
    {
        // Get the primitive action to execute.
        const Action& action = goal->plan.actions[i];

        // Get the group name
        std::string group_name = action.group_name;

        // Pick control group based on group name. HACK
        Group* group = NULL;
        if (group_name == "crichton_left_hand")
            group = &params->left_hand;
        if (group_name == "crichton_right_hand")
            group = &params->right_hand;
        if (group_name == "crichton_left_arm")
            group = &params->left_arm;
        if (group_name == "crichton_right_arm")
            group = &params->right_arm;
        if (group_name == "crichton_torso")
            group = &params->torso;
        if (!group)
        {
            r.error_code = Result::INVALID_GOAL;
            r.error_string = "Failed to get group: " + group_name;
            break;
        }

        // Execute the trajectory.
        r = execute_trajectory(goal->plan.actions[i], action_server, group, params);

        // If something erred, stop further execution.
        if (r.error_code)
            break;

        // Provide feedback.
        {
            Feedback fb;
            fb.progress = (i+1) / (double) goal->plan.actions.size();
            std::stringstream s;
            s << "Finished executing: (" << i << ") " << group_name;
            fb.progress_string = s.str();
            ROS_INFO("%s (%.2f)", fb.progress_string.c_str(), fb.progress);
            action_server->publishFeedback(fb);
        }
    }

    // Clean up memory.
    aa_mem_region_local_release();

    // Print out result.
    if (r.error_code)
        ROS_ERROR_STREAM("Execution failed: " << r.error_string);
    else
        ROS_INFO_STREAM("Execution succeeded!");

    // Return the result to the client.
    if (r.error_code)
        action_server->setAborted(r);
    else
        action_server->setSucceeded(r);
}


int main(int argc, char** argv)
{
    // Initialize.
    sns_init();
    ros::init(argc, argv, "apc_trajectory_bridge");

    // Create node handle in private namespace.
    ros::NodeHandle n("~");

    // Get parameters from server.
    if (!tbridge_params.getParams(n))
    {
        ROS_ERROR("Missing parameters");
        exit(1);
    }

    // Open all channels.
    if (!openChannels(&tbridge_params))
    {
        ROS_ERROR("Failed to open all channels");
        exit(1);
    }

    // Start sns.
    sns_start();

    // Create action server.
    ActionServer server(n, tbridge_params.action_topic, boost::bind(&execute, _1, &server, &tbridge_params), false);
    server.start();

    // Run the main ROS loop.
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

bool setMotorMode(std::string mode_name, enum sns_motor_mode* mode)
{
    if (mode_name == "SNS_MOTOR_MODE_VEL")
        *mode = SNS_MOTOR_MODE_VEL;
    else if (mode_name == "SNS_MOTOR_MODE_POS")
        *mode = SNS_MOTOR_MODE_POS;
    else
    {
        std::cerr << "Unsupported SNS MOTOR MODE: " << mode_name << std::endl;
        exit(1);
    }
    return true;
}

bool Parameters::getParams(ros::NodeHandle& node)
{
    bool success = true;
    std::string key;
    key = "action_topic";
    success &= node.getParam(key, this->action_topic);
    key = "allow_execution";
    success &= node.getParam(key, this->allow_execution);
    key = "max_vel";
    success &= node.getParam(key, this->max_vel);
    key = "max_accel";
    success &= node.getParam(key, this->max_accel);
    key = "left_hand_map";
    success &= node.getParam(key, this->left_hand.map);
    key = "right_hand_map";
    success &= node.getParam(key, this->right_hand.map);
    key = "left_arm_map";
    success &= node.getParam(key, this->left_arm.map);
    key = "right_arm_map";
    success &= node.getParam(key, this->right_arm.map);
    key = "torso_map";
    success &= node.getParam(key, this->torso.map);
    key = "left_hand_state";
    success &= node.getParam(key, this->left_hand.name_state);
    key = "right_hand_state";
    success &= node.getParam(key, this->right_hand.name_state);
    key = "left_arm_state";
    success &= node.getParam(key, this->left_arm.name_state);
    key = "right_arm_state";
    success &= node.getParam(key, this->right_arm.name_state);
    key = "torso_state";
    success &= node.getParam(key, this->torso.name_state);
    key = "left_hand_ref";
    success &= node.getParam(key, this->left_hand.name_ref);
    key = "right_hand_ref";
    success &= node.getParam(key, this->right_hand.name_ref);
    key = "left_arm_ref";
    success &= node.getParam(key, this->left_arm.name_ref);
    key = "right_arm_ref";
    success &= node.getParam(key, this->right_arm.name_ref);
    key = "torso_ref";
    success &= node.getParam(key, this->torso.name_ref);
    std::string name_mode;
    key = "left_hand_mode";
    success &= node.getParam(key, name_mode);
    success &= setMotorMode(name_mode, &this->left_hand.mode);
    key = "right_hand_mode";
    success &= node.getParam(key, name_mode);
    success &= setMotorMode(name_mode, &this->right_hand.mode);
    key = "left_arm_mode";
    success &= node.getParam(key, name_mode);
    success &= setMotorMode(name_mode, &this->left_arm.mode);
    key = "right_arm_mode";
    success &= node.getParam(key, name_mode);
    success &= setMotorMode(name_mode, &this->right_arm.mode);
    key = "torso_mode";
    success &= node.getParam(key, name_mode);
    success &= setMotorMode(name_mode, &this->torso.mode);
    return success;
}
