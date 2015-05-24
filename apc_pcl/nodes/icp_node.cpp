#include <string>
#include <iostream>
#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
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
bool visualize = false;
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
    pcl::console::print_highlight ("Downsampling for registration\n");
    // alp_align(PointCloudT::Ptr object, PointCloudT::Ptr scene, PointCloudT::Ptr object_aligned,
        // int max_iterations, int num_samples, float similarity_thresh, float max_corresp_dist, float inlier_frac)

    // pcl_tools::icp_result result1 = pcl_tools::alp_align(object, scene, object_aligned, 50000, 3, 0.9f, 5.5f * leaf, 0.7f);
    // pcl_tools::visualize(scene, object_aligned);
    // pcl_tools::icp_result result2 = pcl_tools::alp_align(object_aligned, scene, object_aligned, 50000, 3, 0.9f, 7.5f * leaf, 0.4f);
    // pcl_tools::icp_result result3 = pcl_tools::alp_align(object_aligned, scene, object_aligned, 50000, 3, 0.9f, 2.5f * leaf, 0.2f);


    int in_max_iterations = 50000;
    int in_num_samples = 3;
    float in_similarity_thresh = 0.9f;
    float in_max_corresp_dist = 5.5f;
    float in_inlier_frac = 0.7f;
    float in_leaf = 0.005f;
    float feature_radius = 0.02;
    float normal_radius = 0.001;

    //--leaf 0.03 --inlier_frac 0.25 --max_cdist 0.01 --normal_radius 0.05 
    // --feature_radius 0.02 --max_iterations 500000

    /*  int max_iterations, int num_samples, float similarity_thresh, float max_corresp_dist, float inlier_frac, float leaf) */

    result = pcl_tools::alp_align(object, scene, object_aligned, 
        5000000, // Number of iterations
        3, //Number of samples
        0.9f, // similarity threshold
        0.03, // Max corespondence distance
        0.25, // inlier fraction
        0.01 // leaf
    );

    if (visualize) {
        pcl_tools::visualize(object_aligned, scene);
    }

    // Eigen::Affine3d final_affine = result3.affine * result2.affine * result1.affine;
    // Eigen::Affine3d final_affine = result2.affine * result1.affine;

    // pcl_tools::sac_icp(object_aligned, scene);

    // tf::poseEigenToMsg(result.affine, resp.pose);
    tf::poseEigenToMsg(result.affine, resp.pose);
    resp.success = result.converged;
    return true;
}

APC_ICP::APC_ICP(){
    pcl::console::print_highlight ("Initializing ICP Server\n");
    nh.getParam("visualize", visualize);
    nh.getParam("meshpath", meshpath);
    service = nh.advertiseService("/shot_detector", run_icp);
    pcl::console::print_highlight ("--ICP Server Initialized\n");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    APC_ICP *apc_icp(new APC_ICP());
    ros::spin();
}