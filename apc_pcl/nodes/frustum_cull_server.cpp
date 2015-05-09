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
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

#include "ros/ros.h"
#include "../src/pcl_tools/pcl_tools.h"
#include "../src/pcl_tools/pcl_functions.h"
#include "apc_msgs/GetCloudFrustum.h"
#include "apc_msgs/CullCloudBackground.h"
#include "geometry_msgs/Point.h"

class APC_Frust_Cull {

public:
    static bool cull_frustum(apc_msgs::GetCloudFrustum::Request &req, apc_msgs::GetCloudFrustum::Response &resp);
    static bool cull_background(apc_msgs::CullCloudBackground::Request &req, apc_msgs::CullCloudBackground::Response &resp);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud;

    APC_Frust_Cull();
    ros::NodeHandle nh;
    ros::ServiceServer frust_cull_service;
    ros::ServiceServer background_cull_service;

};

bool APC_Frust_Cull::cull_frustum(apc_msgs::GetCloudFrustum::Request &req, apc_msgs::GetCloudFrustum::Response &resp) {
    /* Make this use x, y, w, h 
    It is half-resolution
    */
    std::cout << "Culling Frustum" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PCLPointCloud2 request_cloud;

    pcl_conversions::toPCL(req.cloud, request_cloud);
    pcl::fromPCLPointCloud2(request_cloud, *cloud);

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    std::vector<int> pointIndices;

    for(unsigned int i = (req.x / 2); i < ((req.x / 2) +  (req.width / 2)); i++) {
        for(unsigned int j = (req.x / 2); j < ((req.y / 2) + (req.height / 2)); j++) {
            pointIndices.push_back((req.cloud.width * i) + j);
        }
    }

    boost::shared_ptr< std::vector<int> > indicesptr(new std::vector<int> (pointIndices));
    extract.setInputCloud(cloud);
    extract.setIndices(indicesptr);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    
    // pcl_tools::visualize_cloud(cloud_p);

    pcl::PCLPointCloud2 response_cloud;
    pcl::toPCLPointCloud2(*cloud_p, response_cloud);
    pcl_conversions::fromPCL(response_cloud, resp.sub_cloud);
    std::cout << "Completed frustum cull" << std::endl;

    return true;
}

bool APC_Frust_Cull::cull_background(apc_msgs::CullCloudBackground::Request &req, apc_msgs::CullCloudBackground::Response &resp) {
    /* Remove background service */
    std::cout << "Culling Background" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_clean_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cleaned(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 request_cloud;
    pcl_conversions::toPCL(req.cloud, request_cloud);
    pcl::fromPCLPointCloud2(request_cloud, *cloud_to_clean_rgba);

    pcl::copyPointCloud(*cloud_to_clean_rgba, *cloud_to_clean);

    pcl_functions::removeBackground(cloud_to_clean, background_cloud, cloud_cleaned);
    pcl_tools::visualize_cloud(cloud_cleaned);

    pcl::PCLPointCloud2 response_cloud;
    pcl::toPCLPointCloud2(*cloud_cleaned, response_cloud);
    pcl_conversions::fromPCL(response_cloud, resp.cloud);
    return true;
}

APC_Frust_Cull::APC_Frust_Cull() {
    std::cout << "Initializing" << std::endl;
    frust_cull_service = nh.advertiseService("/cull_frustum", cull_frustum);
    background_cull_service = nh.advertiseService("/cull_background", cull_background);
    std::cout << "advertised" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr APC_Frust_Cull::background_cloud(new pcl::PointCloud<pcl::PointXYZ>);

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    std::cout << "doing anything" << std::endl;
    
    pcl_tools::cloud_from_pcd("../../apc_object_detection/better_background.pcd", *APC_Frust_Cull::background_cloud);

    APC_Frust_Cull *apc_frust_cull(new APC_Frust_Cull());
    ros::spin();
}