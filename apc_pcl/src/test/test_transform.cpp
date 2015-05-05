#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../pcl_tools/loading.cpp"
#include "../pcl_tools/transform.cpp"
#include "../pcl_tools/visualization.cpp"

int main (int argc, char** argv)
{
    /* Test halfspace elimination */
    typedef pcl::PointXYZ cloud_type;
    pcl::console::TicToc time;

    char* filename = argv[1];

    std::cout << "Opening polygon model at " << filename << std::endl;
    pcl::PointCloud<cloud_type>::Ptr output_cloud(new pcl::PointCloud<cloud_type>);
    pcl::PointCloud<cloud_type>::Ptr input_cloud(new pcl::PointCloud<cloud_type>);

    cloud_from_ply(filename, *input_cloud);

    // affine_cloud(Eigen::Vector3f::UnitZ(), -1.5, Eigen::Vector3f(0.4, 0.0, 0.0), *output_cloud, *input_cloud);
    Eigen::Vector3f origin, normal;
    origin << 0, 0, 0;
    normal << 1, 1, 0;

    discard_halfspace(normal, origin, *input_cloud, *output_cloud);

    /* ***** ICP ***** */
    pcl::visualization::PCLVisualizer viewer ("Point Cloud Visualization ENHANCED!");
    viewer.setSize (1280, 1024);  // Visualiser window size

    int v1 (0);
    int v2 (1);
    pcl::visualization::PointCloudColorHandlerCustom<cloud_type> output_cloud_color_h (output_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<cloud_type> input_cloud_color_h (input_cloud, 0, 255, 0);

    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer.addCoordinateSystem (0.04, "View_1", v1);
    viewer.addCoordinateSystem (0.04, "View_2", v2);

    viewer.setBackgroundColor (0, 0, 0, v1);
    viewer.setBackgroundColor (0, 0, 0, v2);

    viewer.addPointCloud(input_cloud, input_cloud_color_h, "input_cloud", v1);
    viewer.addPointCloud(output_cloud, output_cloud_color_h, "output_cloud", v2);
    viewer.addText("0", 10, 10, 50, 1, 1, 1, "info", v2);

    int iterations = 0;
    while(!viewer.wasStopped()) {
        /* Block until done */
        iterations++;
        viewer.spinOnce();
        time.tic();
        std::stringstream ss;
        ss << iterations;
        std::string text = "Iteration: " + ss.str();
        viewer.updateText(text, 10, 16, 60, 0, 1, 0, "info");

        if (iterations > 30) {
            iterations = 0;
        }

        // viewer.updatePointCloud(input_cloud, input_cloud_color_h, "input_cloud");
    }

    return(0);
}