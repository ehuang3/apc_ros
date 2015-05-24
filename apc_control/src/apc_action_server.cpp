#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <apc_msgs/FollowPrimitivePlanAction.h>
#include <Eigen/Dense>
#include <sns.h>
#include <apc_path/Path.h>
#include <apc_path/Trajectory.h>
#include <apc_control/motor_group.h>

using namespace apc_control;

typedef actionlib::SimpleActionServer<apc_msgs::FollowPrimitivePlanAction> ActionServer;
typedef apc_msgs::FollowPrimitivePlanGoalConstPtr GoalConstPtr;
typedef apc_msgs::FollowPrimitivePlanFeedback Feedback;
typedef apc_msgs::FollowPrimitivePlanResult Result;
typedef apc_msgs::PrimitiveAction Action;


MotorGroupError get_time(struct timespec* time)
{
    MotorGroupError r;
    // Get the current time.
    if( clock_gettime( ACH_DEFAULT_CLOCK, time ) )
    {
        SNS_LOG(LOG_ERR, "clock_gettime failed: %s\n", strerror(errno));
        r.error_code = MotorGroupError::FAILED_CLOCK_GETTIME;
        r.error_string = "Failed clock_gettime";
    }
    return r;
}

Result error_to_result(const MotorGroupError& error)
{
    Result r;
    switch (error.error_code)
    {
    case MotorGroupError::STALE_FRAMES:
        r.error_code = Result::STALE_FRAMES;
        break;
    case MotorGroupError::SUCCESSFUL:
        r.error_code = Result::SUCCESSFUL;
        break;
    case MotorGroupError::INVALID_GOAL:
        r.error_code = Result::INVALID_GOAL;
        break;
    case MotorGroupError::INVALID_JOINTS:
        r.error_code = Result::INVALID_JOINTS;
        break;
    case MotorGroupError::OLD_HEADER_TIMESTAMP:
        r.error_code = Result::OLD_HEADER_TIMESTAMP;
        break;
    case MotorGroupError::PATH_TOLERANCE_VIOLATED:
        r.error_code = Result::PATH_TOLERANCE_VIOLATED;
        break;
    case MotorGroupError::GOAL_TOLERANCE_VIOLATED:
        r.error_code = Result::GOAL_TOLERANCE_VIOLATED;
        break;
    case MotorGroupError::FAILED_CLOCK_GETTIME:
        r.error_code = Result::FAILED_CLOCK_GETTIME;
        break;
    case MotorGroupError::PREEMPTED:
        r.error_code = Result::PREEMPTED;
        break;
    case MotorGroupError::TRAJECTORY_DURATION_TOO_LONG:
        r.error_code = Result::TRAJECTORY_DURATION_TOO_LONG;
        break;
    case MotorGroupError::FORCE_TORQUE_DETECTED_CONTACT:
        r.error_code = Result::FORCE_TORQUE_DETECTED_CONTACT;
        break;
    default:
        r.error_code = Result::SUCCESSFUL;
        break;
    }
    r.error_string = error.error_string;
    return r;
}

bool is_action_stationary(const Action& action)
{
    if (action.joint_trajectory.points.size() != 2)
        return false;
    const double tol = 1e-5;
    const trajectory_msgs::JointTrajectoryPoint start = action.joint_trajectory.points.front();
    const trajectory_msgs::JointTrajectoryPoint end = action.joint_trajectory.points.back();
    for (int i = 0; i < action.joint_trajectory.joint_names.size(); i++) {
        if (std::abs(start.positions[i] - end.positions[i]) > tol)
            return false;
    }
    return true;
}

// TODO Assert trajectory is not malformed.
// TODO Force safety limits.
// TODO Haptic profile monitoring.
// TODO Check goal tolerances.
MotorGroupError execute_trajectory(const Action& action,
                                   ActionServer* server,
                                   MotorGroupUnion* motors)
{
    MotorGroupError ret;

    ROS_INFO("Preparing trajectory: %s", action.group_id.c_str());

    if (is_action_stationary(action)) {
        ROS_INFO("Skipping over non-moving trajectory");
        ret.error_string = "Skipping over non-moving trajectory";
        return ret;
    }

    // Get the number of degrees of freedom requested in this action.
    const int n_dof = action.joint_trajectory.joint_names.size();

    // Get the joint order.
    const std::vector<std::string>& joint_names = action.joint_trajectory.joint_names;

    // Reset all motor groups.
    motors->reset();

    // Set the active motor groups.
    if (!motors->setActive(joint_names)) {
        ret.error_code = MotorGroupError::INVALID_JOINTS;
        ret.error_string = "Failed to find matching motor groups for all joints";
        return ret;
    }

    // Check that the joint encoder values are close to the start state.
    Eigen::VectorXd start_position(n_dof);
    const trajectory_msgs::JointTrajectoryPoint& point = action.joint_trajectory.points[0];
    for (int i = 0; i < n_dof; i++)
        start_position[i] = point.positions[i];
    // if (ret = motors->checkStartState(start_position, joint_names))
    //     return ret;

    // Get start state from robot encoders. If we are not executing,
    // the motors will not fill out the state.
    motors->getState(start_position);

    // Convert trajectory into a path, a.k.a. list of eigen vectors.
    std::list<Eigen::VectorXd> P;
    {
        // Add start state.
        P.push_back(start_position);

        // Add the trajectory.
        Eigen::VectorXd p(n_dof);
        const trajectory_msgs::JointTrajectory& T = action.joint_trajectory;
        for (int i = 0; i < T.points.size(); i++)
        {
            // Convert trajectory point to eigen vector.
            for (int j = 0; j < p.rows(); j++)
                p[j] = T.points[i].positions[j];
            P.push_back(p);
        }
    }

    // Create velocity and acceleration limits.
    const Eigen::VectorXd max_vel   = motors->getMaxVel();
    const Eigen::VectorXd max_accel = motors->getMaxAccel();

    // Pass path through Toby's code.
    Trajectory T(Path(P, 0.1), max_vel, max_accel);

    // Abort if the trajectory is not valid.
    if(!T.isValid())
    {
        ret.error_code = MotorGroupError::INVALID_GOAL;
        ret.error_string = "Failed to optimize trajectory";
        return ret;
    }

    // Set trajectory into motor groups.
    motors->setTrajectory(T);

    // Apply pre-conditions.
    {
        if (ret = motors->applyPreConditions())
            return ret;

        // Sleep for 100 ms.
        usleep( (useconds_t) 100 * 1e3 ); // Based on 10x can402 control loop frequency
    }

    ROS_INFO("Executing trajectory: %s", action.group_id.c_str());

    // Get duration of trajectory.
    const double duration = T.getDuration();

    ROS_INFO("Will run for %f seconds", duration);

    if (duration > 40.0) {
        ret.error_code = MotorGroupError::TRAJECTORY_DURATION_TOO_LONG;
        ret.error_string = "Trajectory duration too long";
        return ret;
    }

    // Current timespec.
    struct timespec ts;
    if (ret = get_time( &ts ))
        return ret;

    // Record the start time.
    double start = ((double) ts.tv_sec) + ts.tv_nsec / (double) 1e9;

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
            ret.error_code = MotorGroupError::PREEMPTED;
            ret.error_string = "Trajectory execution preempted";
            break;
        }

        // Check whether we have reached the goal and check whether velocity is zero.
        bool near_goal = motors->isAtGoal();
        bool moving    = motors->isMoving();

        // Stop execution if we are near the goal and stopped moving.
        if (near_goal && !moving)
            break;

        // Get the current time.
        if (ret = get_time( &ts ))
            // On failure, break to ensure post-conditions.
            break;

        // Convert the current time to seconds.
        time = ((double) ts.tv_sec) + ts.tv_nsec / (double) 1e9;

        // Check FT.
        if (ret = motors->checkFTExceeded())
            break;

        // Build reference command messages.
        if (ret = motors->buildCommands(time - start))
            // On failure, break to ensure post-conditions.
            break;

        // Send reference command messages.
        if (ret = motors->sendCommands(time - start))
            // On failure, break to ensure post-conditions.
            break;

        // Sleep for 10 ms.
        usleep( (useconds_t) 10 * 1e3 ); // Based on can402 control loop frequency
    }

    // POST-CONDITION: Apply post-conditions.
    motors->applyPostConditions();

    return ret;
}

void execute(const GoalConstPtr& goal,
             ActionServer* action_server,
             MotorGroupUnion* motors)
{
    // The action request result.
    MotorGroupError r;

    ROS_INFO("Received %lu actions to execute", goal->plan.actions.size());

    // For each primitive action in the primitive plan...
    for (int i = 0; i < goal->plan.actions.size(); i++)
    {
        ROS_INFO("--------------------            EXECUTE            --------------------");

        // Get the primitive action to execute.
        const Action& action = goal->plan.actions[i];

        ROS_INFO( "action name   : %s", action.action_name.c_str());
        ROS_INFO( "action group  : %s", action.group_id.c_str());
        ROS_INFO( "action frame  : %s", action.frame_id.c_str());
        ROS_INFO( "action object : %s", action.object_id.c_str());
        ROS_INFO( "action objkey : %s", action.object_key.c_str());
        ROS_INFO( "action T.size : %ld", action.joint_trajectory.points.size());
        // ROS_INFO( "action type   :", __action_type__(action)c_str());

        // Execute the trajectory.
        r = execute_trajectory(action, action_server, motors);

        // If something erred, stop further execution.
        if (r)
            break;


        ROS_INFO("--------------------             STOP              --------------------");

        // Provide feedback.
        {
            Feedback fb;
            fb.progress = (i+1) / (double) goal->plan.actions.size();
            std::stringstream s;
            s << "Finished executing: (" << i << ") " << action.group_id;
            fb.progress_string = s.str();
            ROS_INFO("%s (%.2f)", fb.progress_string.c_str(), fb.progress);
            action_server->publishFeedback(fb);
        }
    }

    // Print out result.
    if (r)
        ROS_ERROR_STREAM("Execution failed: " << r.error_string);
    else
        ROS_INFO_STREAM("Execution succeeded!");

    // Return the result to the client.
    Result res = error_to_result(r);
    if (res.error_code)
        action_server->setAborted(res);
    else
        action_server->setSucceeded(res);

    ROS_INFO("Foo");
}


int main(int argc, char** argv)
{
    // Initialize.
    sns_init();
    ros::init(argc, argv, "apc_action_server");

    // Create node handle in private namespace.
    ros::NodeHandle n("~");

    // Read in action topic.
    std::string action_topic;
    if (!n.getParam("action_topic", action_topic))
    {
        ROS_ERROR("Failed to get action_topic for apc_action_server");
        exit(1);
    }

    // Create all motor groups.
    MotorGroupUnion* motors = new MotorGroupUnion;

    // Initialize all motor groups.
    if (!motors->initGroups())
        exit(1);

    // Open ACH channels.
    if (!motors->openChannels())
        exit(1);

    // Start sns.
    sns_start();

    // Create action server.
    ActionServer server(n, action_topic, boost::bind(&execute, _1, &server, motors), false);
    server.start();

    // Run the main ROS loop.
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
