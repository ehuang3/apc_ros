#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../pcl_tools/pcl_tools.h"

typedef pcl::PointXYZ InPointType;

int main (int argc, char** argv)
{
    typedef pcl::PointXYZ cloud_type;
    pcl::console::TicToc time;

    char* filename = argv[1];

    std::cout << "Opening polygon model at " << filename << std::endl;
    pcl::PointCloud<cloud_type>::Ptr target_cloud(new pcl::PointCloud<cloud_type>);
    pcl::PointCloud<cloud_type>::Ptr input_cloud(new pcl::PointCloud<cloud_type>);

    pcl_tools::cloud_from_ply(filename, *target_cloud);

    Eigen::Vector3f origin, normal;
    origin << 0, 0, 0;
    normal << 1, 1, 0;

    pcl_tools::discard_halfspace(normal, origin, *target_cloud, *input_cloud);
    pcl_tools::affine_cloud(Eigen::Vector3f::UnitX(), 0.2, Eigen::Vector3f(-0.1, -0.3, 0.0), *input_cloud, *input_cloud);

    pcl::PointCloud<cloud_type>::Ptr input_copy(new pcl::PointCloud<cloud_type>);
    *input_copy = *input_cloud;

    /* ***** ICP ***** */
    pcl::visualization::PCLVisualizer viewer ("Point Cloud Visualization ENHANCED!");
    viewer.setSize (1280, 1024);  // Visualiser window size

    int v1 (0);
    int v2 (1);
    pcl::visualization::PointCloudColorHandlerCustom<InPointType> target_cloud_color_h (target_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<InPointType> input_cloud_color_h (input_cloud, 0, 255, 0);

    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    viewer.setBackgroundColor (0, 0, 0, v1);
    viewer.setBackgroundColor (0, 0, 0, v2);

    viewer.addPointCloud(input_cloud, input_cloud_color_h, "input_cloud", v1);
    viewer.addPointCloud(target_cloud, target_cloud_color_h, "target_cloud", v1);
    viewer.addPointCloud(input_copy, input_cloud_color_h, "input_cloud_copy", v2);

    viewer.addText("0", 10, 10, 50, 1, 1, 1, "info", v2);

    int steps, iterations = 0;
    bool go_on = true;
    while(!viewer.wasStopped()) {
        /* Block until done */
        steps++;
        viewer.spinOnce();
        time.tic();

        if ((steps % 30) == 0) {
            iterations++;
            std::stringstream ss;
            ss << iterations;
            std::string text = "Iteration: " + ss.str();

            pcl_tools::icp_result result;
            if (go_on) {
                result = pcl_tools::apply_icp(input_cloud, target_cloud, 1);
            } else {
                continue;
            }

            std::cout << "Fitness: " << result.fitness << std::endl;
            if (result.fitness < 1e-6) {
                viewer.updateText(text, 10, 16, 60, 0, 1, 0, "info"); 
                go_on = false;
            } else {
                viewer.updateText(text, 10, 16, 60, 1, 0, 0, "info");                
            }
        }

        viewer.updatePointCloud(input_cloud, input_cloud_color_h, "input_cloud");
    }

    return(0);
}