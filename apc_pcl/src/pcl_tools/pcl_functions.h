
#ifndef PCL_FUNCTIONS_H
#define PCL_FUNCTIONS_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_tools.h"
#include <stdlib.h>
#include <Eigen/Core>

namespace pcl_functions{
typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGBA PointType2;

/*!
 * \brief shot_detector::removeBackground
 * \param scene The current scene that the robot sees
 * \param background: The background to be removed
 * This functions takes in a background and current scene pointcloud. It then
 */
void removeBackground(pcl::PointCloud<PointType2>::Ptr scene, pcl::PointCloud<PointType2>::Ptr background, pcl::PointCloud<PointType2>::Ptr pc_back_subtracted)
{
    // We use a kdtree to search for points that are close enough to the other pointcloud
    pcl::KdTreeFLANN<PointType2> kdtree;

    kdtree.setInputCloud (background);
    //The distance we use to compare if a point in one pc is the same as a point in the other pc
    float dist=.0005;
    std::vector<int> indices;
    //Remove NaN values
    pcl::PointCloud<PointType2>::Ptr nan_scene(new pcl::PointCloud<PointType2>());
    pcl::removeNaNFromPointCloud(*scene, *nan_scene, indices);
    std::vector<int> indexes;
    //  Iterate through pc and compare to points in the kdtree
    for (size_t i = 0; i < nan_scene->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        int found_neighs = kdtree.nearestKSearch (nan_scene->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < dist)
        {
            indexes.push_back(i);
        }
    }
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    inliers->indices=indexes;
    //Extract the indices that are not in the scene and the background(this is the pointcloud
    // with the background subtracted)
    pcl::ExtractIndices<PointType2> eifilter (false);
    eifilter.setInputCloud(nan_scene);
    eifilter.setIndices(inliers);
    eifilter.setNegative (true);
    eifilter.filter(*pc_back_subtracted);
}

enum AXIS{X,Y,Z};
void filter(pcl::PointCloud<PointType>::Ptr scene, pcl::PointCloud<PointType>::Ptr filtered_scene, float range1, float range2, AXIS axis)
{
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (scene);
    switch(axis){
    case X:
        pass.setFilterFieldName("x");
        break;
    case Y:
        pass.setFilterFieldName("y");
        break;
    case Z:
        pass.setFilterFieldName("z");
        break;
    }
    pass.setFilterLimits (range1, range2);
    pass.filter (*filtered_scene);

}
void voxelFilter(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr filtered_cloud, float sample_size)
{
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(sample_size,sample_size,sample_size);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*filtered_cloud);
}
/*!
 * \brief convertMsg
 * \param data_msg
 * \param cloud = the converted output cloud
 * Takes in a ros poincloud2 msg and converts it
 */
void convertMsg(sensor_msgs::PointCloud2 data_msg, pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(data_msg, pcl_pc);
    pcl::PointCloud<PointType>::Ptr image_cloud(new pcl::PointCloud<PointType> ());
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);
}
}

// PCL_FUNCTIONS_H
#endif

