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
#include <eigen_conversions/eigen_msg.h>
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
    // static pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud;
    static void recolor(pcl::PointCloud<pcl::PointXYZRGBA> &mycloud, int x, int y, int h, int w);

    APC_Frust_Cull();
    ros::NodeHandle nh;
    ros::ServiceServer frust_cull_service;
    ros::ServiceServer background_cull_service;
};

void APC_Frust_Cull::recolor(pcl::PointCloud<pcl::PointXYZRGBA> &mycloud, int x, int y, int h, int w) {

    typename pcl::PointCloud<pcl::PointXYZRGBA>::iterator point;

    for (point = mycloud.points.begin(); point < mycloud.points.end(); point++) {
        if ((point->x <= x + w) && (point->x > x)) {
            if ((point->y <= y + h) && (point->y > y)) {
                point->rgba = pcl_tools::pcl_color(255, 0, 0);
            }
        }
    }
    std::cout << "filcom" << std::endl;
}

bool APC_Frust_Cull::cull_frustum(apc_msgs::GetCloudFrustum::Request &req, apc_msgs::GetCloudFrustum::Response &resp) {
    /* Make this use x, y, w, h 
    It is half-resolution
    */
    std::cout << "Culling Frustum" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::cout << "Culling Frustum" << std::endl;
    pcl::fromROSMsg(req.cloud, *cloud);

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

    std::vector<int> pointIndices;
    // pcl_tools::visualize_cloud(cloud);

    std::cout << "Culling Frustum" << std::endl;
    for(unsigned int i = (req.x / 2); i < ((req.x / 2) +  (req.width / 2)); i++) {
        for(unsigned int j = (req.x / 2); j < ((req.y / 2) + (req.height / 2)); j++) {
            pointIndices.push_back((req.cloud.width * i) + j);
        }
    }

    // std::cout << request_cloud.width << " " << request_cloud.height << std::endl;
    // for(unsigned int i = (req.x / 2); i < ((req.x / 2) +  (req.width / 2)); i++) {
    //     for(unsigned int j = (req.x / 2); j < ((req.y / 2) + (req.height / 2)); j++) {
    //         pointIndices.push_back((req.cloud.width * i) + j);
    //     }
    // }

    std::cout << "Culling Frustum" << std::endl;
    boost::shared_ptr< std::vector<int> > indicesptr(new std::vector<int> (pointIndices));
    extract.setInputCloud(cloud);
    extract.setIndices(indicesptr);
    extract.setNegative(false);
    std::cout << "Culling Frustum" << std::endl;

    extract.filter(*cloud_p);

    std::cout << "Culling Frustum" << std::endl;

    // recolor(*cloud_p, req.x / 2, req.y / 2, req.height / 2, req.width / 2);
    
    std::cout << "Culling Frustum" << std::endl;
    pcl_tools::visualize(cloud_p);

    pcl::toROSMsg(*cloud_p, resp.sub_cloud);
    std::cout << "Completed frustum cull" << std::endl;

    return true;
}

bool APC_Frust_Cull::cull_background(apc_msgs::CullCloudBackground::Request &req, apc_msgs::CullCloudBackground::Response &resp) {
    /* Remove background service */
    std::cout << "Culling Background" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_clean_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cleaned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.cloud, *cloud_to_clean);

    pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.cloud_background, *background_cloud);

    // pcl_tools::visualize_cloud(cloud_to_clean);
    pcl_functions::removeBackground(cloud_to_clean, background_cloud, cloud_cleaned);
    pcl_tools::visualize(cloud_cleaned);

    pcl::toROSMsg(*cloud_cleaned, resp.cloud);
    std::cout << "Culled Background" << std::endl;

    return true;
}

APC_Frust_Cull::APC_Frust_Cull() {
    std::cout << "Initializing" << std::endl;
    frust_cull_service = nh.advertiseService("/cull_frustum", cull_frustum);
    background_cull_service = nh.advertiseService("/cull_background", cull_background);
    std::cout << "advertised" << std::endl;
}

// pcl::PointCloud<pcl::PointXYZ>::Ptr APC_Frust_Cull::background_cloud(new pcl::PointCloud<pcl::PointXYZ>);

int main(int argc, char** argv) {
    ros::init(argc, argv, "frustum_cull_node");
    std::cout << "doing anything" << std::endl;
    
    // pcl_tools::cloud_from_pcd("../../apc_object_detection/better_background.pcd", *APC_Frust_Cull::background_cloud);

    APC_Frust_Cull *apc_frust_cull(new APC_Frust_Cull());
    ros::spin();
}