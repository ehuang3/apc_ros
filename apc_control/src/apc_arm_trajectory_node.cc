#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sns.h>


typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

typedef control_msgs::FollowJointTrajectoryGoalConstPtr GoalPtr;

ach_channel_t chan_path;

bool allow_trajectory_execution;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{
	std::cout << goal->trajectory << std::endl;

	// Get the number of dofs.
	const int num_dofs = goal->trajectory.joint_names.size();

	// Get the number of steps.
	const int num_steps = goal->trajectory.points.size();

	// Allocate space for a sns dense path message.
	struct sns_msg_path_dense* msg = sns_msg_path_dense_alloc(num_steps, num_dofs);

	// Fill the message header.
	sns_msg_header_fill( &msg->header );

	// Copy joint trajectory over to message.
	int counter = 0;
	for (int i = 0; i < num_steps; i++)
	{
		for (int j = 0; j < num_dofs; j++)
		{
			msg->x[counter++] = goal->trajectory.points[i].positions[j];
		}
	}

	// Status.
	ach_status_t r = ACH_OK;

	// Send the trajectory.
	if (allow_trajectory_execution)
		r = ach_put( &chan_path, msg, sns_msg_path_dense_size(msg) );

	// set success.
	if (r == ACH_OK)
		as->setSucceeded();

	// Free sns dense path message from memory.
	free(msg);
}


int main(int argc, char** argv)
{
	sns_init();

	ros::init(argc, argv, "apc_arm_trajectory_node");
	ros::NodeHandle n("~");

	std::string topic = "/crichton/left_arm/controller";
	n.getParam("topic", topic);

	std::string chan_name = "ref-left";
	n.getParam("channel", chan_name);

	allow_trajectory_execution = true;
	n.getParam("allow_trajectory_execution", allow_trajectory_execution);

	sns_chan_open( &chan_path,  chan_name.c_str(),  NULL );

	sns_start();

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
