#ifndef SPINIMAGE_H
#define SPINIMAGE_H

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>

class SpinImage
{

    typedef pcl::Histogram<153> SpinImage_type;
public:
    SpinImage();
    void createImage(std::string filename);
    void createImage(pcl::PointCloud<pcl::PointXYZ> keypoints);
    void createImage(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints);
    void calcDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage_type> si;
    pcl::PointCloud<SpinImage_type>::Ptr descriptors;
};

#endif // SPINIMAGE_H
