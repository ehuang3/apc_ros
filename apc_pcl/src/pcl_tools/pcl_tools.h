#pragma once
#include <string>
#include <iostream>
#include <pcl/point_types.h>


namespace pcl_tools {
    /* Loading */
    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud);
    bool cloud_from_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);
    bool cloud_from_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud);
    bool cloud_from_stl(std::string filename, pcl::PCLPointCloud2& cloud);

    /* Registration */
    struct icp_result {
        bool converged;
        double fitness;
        Eigen::Affine3d affine;
    };

    icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int);
    icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
        Eigen::Vector3f axis, float angle, Eigen::Vector3f translation_offset, int);
    icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr offset_cloud, Eigen::Vector3f axis, float angle, Eigen::Vector3f translation_offset, int iterations);

    icp_result sac_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int);

    /* Visualization */
    void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
    void visualize_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud);
    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2);
    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    void color_cloud(int pcl_color, pcl::PointCloud<pcl::PointXYZ> &input_cloud, pcl::PointCloud<pcl::PointXYZRGBA> &destination_cloud);
    int pcl_color(float r, float g, float b);

    /* Transformation */
    void discard_halfspace(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
        pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud);
    void discard_halfspace(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
        pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud);
    void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud);
    void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud);
}
