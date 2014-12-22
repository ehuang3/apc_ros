#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <sns.h>


// typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> Server;
// typedef control_msgs::GripperCommandGoalConstPtr GoalPtr;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
typedef control_msgs::FollowJointTrajectoryGoalConstPtr GoalPtr;

ach_channel_t chan_ref;

std::vector<std::string> joint_names;


void execute(const GoalPtr& goal, Server* as)
{
	std::cout << *goal << std::endl;

	// Get the number of dofs.
	const int num_dofs = goal->trajectory.joint_names.size();
	ROS_ASSERT(num_dofs == joint_names.size());

	// Get the number of steps.
	const int num_steps = goal->trajectory.points.size();

	// Map trajectory joint order to motor reference order.
	std::map<int, int> joint_map;
	for (int i = 0; i <	num_dofs; i++)
	{
		joint_map[i] = std::find(goal->trajectory.joint_names.begin(),
								 goal->trajectory.joint_names.end(),
								 joint_names[i]) - goal->trajectory.joint_names.begin();
	}

	// Allocate 7 joints for a sns motor reference message.
	struct sns_msg_motor_ref* msg =	sns_msg_motor_ref_local_alloc(7);

	// Fill the message header.
	sns_msg_header_fill( &msg->header );

	// Set the control mode.
	msg->mode = SNS_MOTOR_MODE_POS;

	// HACK Copy the last joint position in the trajectory because we
	// don't want to use the RRT generated trajectory.
	for (int i = 0; i <	7; i++)
	{
		msg->u[i] =	goal->trajectory.points[num_steps - 1].positions[joint_map[i]];
	}

	// HACK	Move the valid timestamp of the	motor reference command forward.
	struct timespec now;
	if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
		SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
	}
	int64_t valid_ns = (int64_t)((1000000000) / 1);
	sns_msg_set_time( &msg->header, &now, valid_ns ); // valid_ns value taken from piranha/src/pirctrl.c

	// Send the reference command.
	ach_status_t r;
	r = ach_put( &chan_ref, msg, sns_msg_motor_ref_size(msg) );

	// set success.
	if (r == ACH_OK)
		as->setSucceeded();

	// Free sns motor reference message from memory.
	aa_mem_region_local_release();
}


int main(int argc, char** argv)
{
	sns_init();

	ros::init(argc, argv, "apc_gripper_trajectory_server");
	ros::NodeHandle n("~");

	std::string side = "left";
	n.getParam("side", side);

	std::string topic = "/crichton/left_hand/controller";
	n.getParam("topic", topic);

	std::string channel = "sdhref-left";
	n.getParam("channel", channel);

	sns_chan_open( &chan_ref,  channel.c_str(),  NULL );

	sns_start();

	// Joint names.
    joint_names.push_back("knuckle_joint");
    joint_names.push_back("finger_12_joint");
    joint_names.push_back("finger_13_joint");
    joint_names.push_back("thumb_2_joint");
    joint_names.push_back("thumb_3_joint");
	joint_names.push_back("finger_22_joint");
    joint_names.push_back("finger_23_joint");

	// Fill out joint state names.
	for (int i = 0; i < 7; i++)
	{
		std::stringstream ss;
		ss << "crichton_" << side << "_" << joint_names[i];
		joint_names[i] = ss.str();
	}

	Server server(n, topic, boost::bind(&execute, _1, &server), false);
	server.start();

	// TODO Find a better polling method.
	ros::Rate rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}
