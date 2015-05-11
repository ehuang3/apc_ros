#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../pcl_tools/pcl_tools.h"
// #include "../pcl_tools/loading.cpp"
// #include "../pcl_tools/transform.cpp"
// #include "../pcl_tools/visualization.cpp"
// #include "../pcl_tools/registration.cpp"

typedef pcl::PointXYZ InPointType;

int main (int argc, char** argv)
{
    typedef pcl::PointXYZ cloud_type;

    char* filename = argv[1];
    pcl::console::TicToc time;

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

    int frames, iterations = 0;
    bool go_on = true;

    // Eigen::Vector3f origin(0.0, 0.0, 0.0);
     static const Eigen::Vector3f seed_position_arr[] = {
        Eigen::Vector3f(-1, 0, 0),
        Eigen::Vector3f(0, -1, 0),
        Eigen::Vector3f(0, 0, -1),
        Eigen::Vector3f(1, 0, 0),
        Eigen::Vector3f(0, 1, 0),
        Eigen::Vector3f(0, 0, 1)
    };

    std::vector<Eigen::Vector3f> seed_positions (seed_position_arr, seed_position_arr + sizeof(seed_position_arr) / sizeof(seed_position_arr[0]));
    Eigen::ArrayXf fitnesses = Eigen::ArrayXf::Zero(36);
    int position = 0;
    int max_position = seed_positions.size();

    // apply_icp(input_cloud, target_cloud, Eigen::Vector3f::UnitZ(), 0.0, *position)

    pcl::PointCloud<pcl::PointXYZ>::Ptr offset_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    while(!viewer.wasStopped()) {
        /* Block until done */
        frames++;
        viewer.spinOnce();
        time.tic();

        if ((frames % 30) == 0) {
            iterations++;
            std::stringstream ss;
            ss << iterations;
            std::string text = "Iteration: " + ss.str();

            pcl_tools::icp_result result;
            if (go_on) {
                result = pcl_tools::apply_icp(input_cloud, target_cloud, offset_cloud, Eigen::Vector3f::UnitZ(), 0.0, seed_positions[position], 1);
            } else {
                if (position >= max_position) {
                    continue;
                }
                iterations = 0;
                position++;
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