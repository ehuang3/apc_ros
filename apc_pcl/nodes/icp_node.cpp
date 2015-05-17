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
#include <pcl/filters/voxel_grid.h>
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
    APC_ICP();
    ros::NodeHandle nh;
    ros::ServiceServer service;
};

bool APC_ICP::run_icp(apc_msgs::shot_detector_srv::Request &req, apc_msgs::shot_detector_srv::Response &resp) {
    /* Run the Amazon Picking Challenge IPC object pose refinement method */

    std::cout << "Attempting to ICP!" << std::endl;
    PointCloudT::Ptr scene(new PointCloudT);
    pcl::fromROSMsg(req.pointcloud, *scene);

    PointCloudT::Ptr object(new PointCloudT);
    pcl::fromROSMsg(req.targetcloud, *object);

    PointCloudT::Ptr object_aligned(new PointCloudT);

    pcl_tools::icp_result result;
    // pcl_tools::visualize(scene, object);
    // result = pcl_tools::apply_icp(scene, object, 45);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*scene, *scene, indices);
    pcl::removeNaNFromPointCloud(*object, *object, indices);

    // Downsample
    pcl::console::print_highlight ("Downsampling...\n");
    pcl::VoxelGrid<pcl::PointNormal> grid;
    const float leaf = 0.005f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);
    grid.setInputCloud (scene);
    grid.filter (*scene);

    pcl_tools::icp_result result1 = pcl_tools::alp_align(object, scene, object_aligned, 50000, 3, 0.9f, 5.5f * leaf, 0.7f);
    pcl_tools::icp_result result2 = pcl_tools::alp_align(object_aligned, scene, object_aligned, 50000, 3, 0.9f, 7.5f * leaf, 0.4f);
    pcl_tools::icp_result result3 = pcl_tools::alp_align(object_aligned, scene, object_aligned, 50000, 3, 0.9f, 2.5f * leaf, 0.2f);

    Eigen::Affine3d final_affine = result1.affine * result2.affine * result3.affine;

    pcl_tools::visualize(scene, object);
    // pcl_tools::visualize_cloud(scene);
    // tf::poseEigenToMsg(result.affine, resp.pose);
    tf::poseEigenToMsg(final_affine, resp.pose);


    return true;
}

APC_ICP::APC_ICP(){
    std::cout << "Initializing icp server" << std::endl;
    nh.getParam("meshpath", meshpath);
    std::cout << "meshpath: " << meshpath << std::endl;
    service = nh.advertiseService("/shot_detector", run_icp);
    std::cout << "advertised icp server" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    std::cout << "doing anything" << std::endl;
    APC_ICP *apc_icp(new APC_ICP());

    ros::spin();
}