#include <string>
#include <iostream>
#include "ros/ros.h"
#include "../src/pcl_tools/registration.cpp"
#include "apc_msgs/RunICP.h"
#include "geometry_msgs/PoseStamped.h"

class APC_ICP {

public:
    static bool run_ICP(apc_msgs::RunICP::Request &req, apc_msgs::RunICP::Response &resp);
    void clicked_point_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    APC_ICP();
    ros::NodeHandle nh;
    ros::ServiceServer service;

};

bool APC_ICP::run_ICP(apc_msgs::RunICP::Request &req, apc_msgs::RunICP::Response &resp) {
    /* Run the Amazon Picking Challenge IPC object pose refinement method */

    std::cout << "Attempting to ICP!" << std::endl;
    std::cout << "Frameid " << req.cloud.header.frame_id << std::endl;
    resp.object_pose.pose.position.x = 0.0;
    resp.object_pose.pose.position.y = 0.0;
    resp.object_pose.pose.position.z = 0.0;
    resp.object_pose.pose.orientation.x = 0.0;
    resp.object_pose.pose.orientation.y = 0.0;
    resp.object_pose.pose.orientation.z = 0.0;
    resp.object_pose.pose.orientation.w = 0.0;

    // ---------> TO FIX
    resp.object_pose.header.frame_id = "/camera";
    return true;
}

APC_ICP::APC_ICP(){
    std::cout << "Initializing" << std::endl;
    service = nh.advertiseService("/runICP", run_ICP);
    // ros::Subscriber sub = nh.subscribe("clicked_point", 1, clicked_point_callback);
    std::cout << "advertised" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    std::cout << "doing anything" << std::endl;
    APC_ICP *apc_icp(new APC_ICP());

    ros::spin();
}