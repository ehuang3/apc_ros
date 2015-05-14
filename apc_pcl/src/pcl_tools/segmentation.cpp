#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include "pcl_tools.h"

namespace pcl_tools {
void extract_indices(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, pcl::PointIndices &cluster) {
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(input_cloud);
    pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(cluster));
    extract.setIndices(cluster_ptr);
    extract.setNegative(false);
    extract.filter(*output_cloud);
}

void segment_region_growing(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, int index, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud) {

    pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr< pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);

    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");  // Filter the huge quantity of points at the origin that mean nothing
    pass.setFilterLimits (-0.01, 0.01);

    // pass.setFilterFieldName ("y");
    // pass.setFilterLimits (-0.1, 0.1);
    // pass.setFilterFieldName ("x");
    // pass.setFilterLimits (-0.1, 0.1);

    pass.setFilterLimitsNegative(true);
    pass.filter (*indices);

    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr temp_visualization (new pcl::PointCloud <pcl::PointXYZRGBA>);
    pcl::PointIndices filter_pts;

    pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices());
    cluster_ptr->indices = *indices;

    extract_indices(cloud, temp_visualization, *cluster_ptr);

    pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
    reg.setMinClusterSize (10);
    reg.setMaxClusterSize (100000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setIndices(indices);
    reg.setInputCloud (cloud);

    reg.setInputNormals (normals);
    // reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setSmoothnessThreshold (15.0 / 180.0 * M_PI);  // More relaxed angle constraints
    reg.setCurvatureThreshold (1.0);

    // std::vector <pcl::PointIndices> clusters;
    // reg.extract (clusters);

    /*
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr colored_cloud_rgba(new pcl::PointCloud <pcl::PointXYZRGBA>);
    pcl::copyPointCloud(*colored_cloud, *colored_cloud_rgba);
    visualize(colored_cloud_rgba, "colored segmentation");
    */

    pcl::PointIndices::Ptr cluster(new pcl::PointIndices());
    reg.getSegmentFromPoint(index, *cluster);
    extract_indices(cloud, output_cloud, *cluster);
}
}