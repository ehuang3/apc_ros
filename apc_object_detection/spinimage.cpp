#include "spinimage.h"

SpinImage::SpinImage()
{
}

void SpinImage::createImage(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);;
    pcl::PCLPointCloud2 test;
    pcl::PLYReader reader;
    if (reader.read(filename,test) != 0)
    {
    };
    pcl::fromPCLPointCloud2(test, *cloud);
    /*pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    }*/
    calcDescriptors(cloud);
}

void SpinImage::createImage(pcl::PointCloud<pcl::PointXYZ> keypoints)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&keypoints);
    calcDescriptors(cloud);
}

void SpinImage::createImage(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints)
{
    calcDescriptors(keypoints);
}

void SpinImage::calcDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
     // Object for storing the spin image for each point.
     //pcl::PointCloud<SpinImage_type>::Ptr temp_descriptors(new pcl::PointCloud<SpinImage>());
        std::cerr  << "Starting2" << std::endl;
     // Note: you would usually perform downsampling now. It has been omitted here
      // for simplicity, but be aware that computation can take a long time.
     // Estimate the normals.
     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
     normalEstimation.setInputCloud(cloud);
     normalEstimation.setRadiusSearch(0.03);
     pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
     normalEstimation.setSearchMethod(kdtree);
     normalEstimation.compute(*normals);

     si.setInputCloud(cloud);
     si.setInputNormals(normals);
     pcl::PointCloud<pcl::Histogram<153> >::Ptr temp_descriptors (new pcl::PointCloud<pcl::Histogram<153> >);
     // Radius of the support cylinder.
     si.setRadiusSearch(0.02);
     // Set the resolution of the spin image (the number of bins along one dimension).
     // Note: you must change the output histogram size to reflect this.
     si.setImageWidth(8);

     si.compute(*temp_descriptors);
     descriptors=temp_descriptors;
}
