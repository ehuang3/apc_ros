#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>


typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> Server;

typedef control_msgs::GripperCommandGoalConstPtr GoalPtr;


void execute(const control_msgs::GripperCommandGoalConstPtr& goal, Server* as)
{
	std::cout << "Received Gripper Command:" << std::endl;

	std::cout << goal->command << std::endl;

	as->setSucceeded();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "APC Gripper Trajectory Server");
	ros::NodeHandle n("~");
	std::string topic = "";
	n.getParam("topic", topic);
	// std::cout << n.hasParam("topic");
	// std::cout << topic << std::endl;
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
