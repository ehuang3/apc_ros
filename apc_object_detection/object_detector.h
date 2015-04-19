#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/search/kdtree.h>
#include "spinimage.h"
#include <iostream>

class object_detector
{
    typedef pcl::Histogram<153> SpinImage_type;
public:
    object_detector();
    ros::NodeHandle nh;
    ros::Subscriber kinect;

    void processImage();
    void PointCloudCallback(sensor_msgs::PointCloud2 data_msg);

    pcl::PointCloud<pcl::PointXYZ> image_cloud;

private:
    SpinImage model;
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
    pcl::PointCloud<pcl::PointXYZ>::Ptr detectKeyPoints();
    void matchKeyPoints(SpinImage scene);
    double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif // OBJECT_DETECTOR_H
