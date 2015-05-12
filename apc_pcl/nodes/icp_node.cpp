#include <string>
#include <iostream>
#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <eigen_conversions/eigen_msg.h>
#include "apc_msgs/GetCloudFrustum.h"
#include "apc_msgs/CullCloudBackground.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "apc_msgs/shot_detector_srv.h"
#include "geometry_msgs/PoseStamped.h"
#include "../src/pcl_tools/pcl_functions.h"
#include "../src/pcl_tools/pcl_tools.h"

std::string meshpath;
class APC_ICP {

public:
    static bool run_icp(apc_msgs::shot_detector_srv::Request &req, apc_msgs::shot_detector_srv::Response &resp);
    void clicked_point_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    APC_ICP();
    ros::NodeHandle nh;
    ros::ServiceServer service;
};

bool APC_ICP::run_icp(apc_msgs::shot_detector_srv::Request &req, apc_msgs::shot_detector_srv::Response &resp) {
    /* Run the Amazon Picking Challenge IPC object pose refinement method */

    std::cout << "Attempting to ICP!" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.pointcloud, *target_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_tools::cloud_from_ply(meshpath, *input_cloud);
    pcl_tools::icp_result result;
    result = pcl_tools::apply_icp(input_cloud, target_cloud, 45);
    // pcl_tools::visualize_cloud(input_cloud);
    pcl_tools::visualize(input_cloud, target_cloud);
    tf::poseEigenToMsg(result.affine, resp.pose);

    return true;
}

APC_ICP::APC_ICP(){
    std::cout << "Initializing" << std::endl;
    nh.getParam("meshpath", meshpath);
    std::cout << "meshpath: " << meshpath << std::endl;
    service = nh.advertiseService("/shot_detector", run_icp);
    std::cout << "advertised" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    std::cout << "doing anything" << std::endl;
    APC_ICP *apc_icp(new APC_ICP());

    ros::spin();
}