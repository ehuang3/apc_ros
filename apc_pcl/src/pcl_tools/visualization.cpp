#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/console/time.h>   // TicToc
#include "pcl_tools.h"

namespace pcl_tools {
void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {
    /* Run pcl visualize on an XYZ point cloud
        X:Red
        Y:Green
        Z:Blue
        --> Add the ability to close this
    */
    pcl::visualization::CloudViewer viewer ("Point Cloud Visualization");
    viewer.showCloud(pointCloud);
    while(!viewer.wasStopped()) {/* Block until done */}
    // viewer.close();
}

void visualize_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud) {
    /* Run pcl visualize on an XYZR point cloud
    ISSUES:
        Couldn't figure out how to template this
    Run pcl visualize on an XYZ point cloud
        X:Red
        Y:Green
        Z:Blue
    */
    pcl::visualization::CloudViewer viewer ("Point Cloud Visualization");
    viewer.showCloud(pointCloud);
    while(!viewer.wasStopped()) {/* Block until done */}
    // viewer.close();
}

void color_cloud(int pcl_color, pcl::PointCloud<pcl::PointXYZ> &input_cloud, pcl::PointCloud<pcl::PointXYZRGBA> &destination_cloud) {
    /* Set the color of a cloud. Output is RGBA 
        Speed:
            - Should preallocate an XYZRGBA cloud the size of input_cloud and not use push_back
                Will this help? -->TODO
    */
    typename pcl::PointCloud<pcl::PointXYZ>::iterator point;
    pcl::PointXYZRGBA new_point;

    for (point = input_cloud.points.begin(); point < input_cloud.end(); point++) {
        new_point.x = point->x;
        new_point.y = point->y;
        new_point.z = point->z;
        new_point.rgba = pcl_color;
        destination_cloud.push_back(new_point);
    }
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::visualization::PCLVisualizer::Ptr viewer) {
    if (viewer) { // Points to something
        viewer->addPointCloud(cloud);
    }
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    /* Visualize with default arguments */
    visualize(cloud, "Cloud Visualization");
}

void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    /* Visualize with default arguments */
    visualize(cloud, "Cloud Visualization");
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2) {
    /* Visualize with default arguments */
    visualize(cloud_1, cloud_2, "Cloud Visualization");
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string window_name) {
    /* Visualize XYZ with a single cloud */
    pcl::visualization::PCLVisualizer viewer(window_name);
    viewer.setSize(500, 500);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h (cloud, 255, 0, 0);
    viewer.addCoordinateSystem (0.04, "View_1");
    viewer.addPointCloud(cloud, cloud_color_h, "cloud");
    while(!viewer.wasStopped()) {
        viewer.spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 

    }
    viewer.close();
}

void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string window_name) {
    /* Visualize XYZRGBA with a single cloud */
    pcl::visualization::PCLVisualizer viewer(window_name);
    viewer.setSize(500, 500);
    viewer.addPointCloud(cloud, "cloud");
    viewer.addCoordinateSystem (0.04, "View_1");
    while(!viewer.wasStopped()) {
        viewer.spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
    }
    viewer.close();
}

void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_2, std::string window_name) {
    /* Visualize XYZRGBA with two clouds */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_1, *cloud_1_xyz);
    pcl::copyPointCloud(*cloud_2, *cloud_2_xyz);
    visualize(cloud_1_xyz, cloud_2_xyz, window_name);
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2, std::string window_name) {
    /* Visualize XYZ with two clouds */
    pcl::visualization::PCLVisualizer viewer(window_name);
    viewer.setSize(500, 500);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_1_color_h (cloud_1, 255, 0, 0);
    viewer.addPointCloud(cloud_1, cloud_1_color_h, "cloud_1");
    viewer.addText("Cloud 1", 10, 16, 50, 1, 0, 0, "info");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_2_color_h (cloud_2, 0, 255, 0);
    viewer.addPointCloud(cloud_2, cloud_2_color_h, "cloud_2");
    viewer.addCoordinateSystem (0.04, "View_1");

    while(!viewer.wasStopped()) {
        viewer.spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
    }
    viewer.close();
}

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandlerT;

void visualize(PointCloudT::Ptr scene, PointCloudT::Ptr object) {
    /* Visualize PointCloudT with two clouds */
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object, ColorHandlerT (object, 0.0, 0.0, 255.0), "object_aligned");
    visu.addCoordinateSystem (0.04, "View_1");

    visu.spin ();
}

void visualize(PointCloudT::Ptr scene, std::string window_name) {
    /* Visualize PointCloudT with one clouds */
    pcl::visualization::PCLVisualizer visu(window_name);
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addCoordinateSystem (0.04, "View_1");
    visu.spin ();
}

int pcl_color(float r, float g, float b) {
    /* Get the PCL style bit-magic color specification */
    return ((int)r) << 16 | ((int)g) << 8 | ((int)b);
}
}