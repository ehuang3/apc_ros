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

#define xyToLinear(x, y, width) ((width * y) + x)

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
    /* This was a test function for coloring the section of the point cloud we are removing
        As is now obvious, this does not work because it uses spatial coordinates, and not image coordinates */
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
    std::cout << "Width " << req.cloud.width << " height " << req.cloud.height << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromROSMsg(req.cloud, *input_cloud);
    if (!req.region_growing) {
        std::cout << "Culling Frustum" << std::endl;
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

        std::vector<int> pointIndices;
        for(unsigned int i = (req.x); i < ((req.x) +  (req.width)); i++) {
            for(unsigned int j = (req.y); j < ((req.y) + (req.height)); j++) {
                pointIndices.push_back(xyToLinear(i, j, req.cloud.width));
            }
        }

        boost::shared_ptr< std::vector<int> > indicesptr(new std::vector<int> (pointIndices));
        extract.setInputCloud(input_cloud);
        extract.setIndices(indicesptr);
        extract.setNegative(false);

        extract.filter(*output_cloud);
        pcl_tools::visualize(output_cloud, "frustum culled");
        pcl::toROSMsg(*output_cloud, resp.sub_cloud);
        std::cout << "Completed frustum cull" << std::endl;
        return true;
    } else {
        std::cout << "Culling using region growing method" << std::endl;

        int centroid_x, centroid_y;
        centroid_x = (req.width / 2) + req.x;
        centroid_y = (req.height / 2) + req.y;
        int seed_index = xyToLinear(centroid_x, centroid_y, req.cloud.width);
        pcl_tools::segment_region_growing(input_cloud, seed_index, output_cloud);
        pcl_tools::visualize(output_cloud, "Culled with region growing");
        std::cout << "Culled" << std::endl;
        // pcl::io::savePCDFile ("/home/apc/repos/apc/src/apc_ros/niko_file.pcd", *output_cloud, true);
        pcl::toROSMsg(*output_cloud, resp.sub_cloud);

        return true;
    }
}

bool APC_Frust_Cull::cull_background(apc_msgs::CullCloudBackground::Request &req, apc_msgs::CullCloudBackground::Response &resp) {
    /* Remove background service */
    std::cout << "Culling Background" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_clean_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cleaned(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(req.cloud, *cloud_to_clean);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr background_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(req.cloud_background, *background_cloud);
    // pcl_tools::visualize(background_cloud, cloud_to_clean, "Background image");
    pcl_functions::removeBackground(cloud_to_clean, background_cloud, cloud_cleaned);
    // pcl_tools::visualize(cloud_cleaned, "Cloud after cleaning");

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


int main(int argc, char** argv) {
    ros::init(argc, argv, "frustum_cull_node");
    std::cout << "doing anything" << std::endl;
    
    APC_Frust_Cull *apc_frust_cull(new APC_Frust_Cull());
    ros::spin();
}