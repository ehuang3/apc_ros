#pragma once
#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

namespace pcl_tools {

    /* Loading */
    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud);
    bool cloud_from_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);
    bool cloud_from_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud);
    bool cloud_from_stl(std::string filename, pcl::PCLPointCloud2& cloud);
    bool cloud_from_stl(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud);
    bool cloud_from_stl(std::string filename, pcl::PointCloud<pcl::PointNormal>& cloud);
    bool cloud_from_stl(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud);

    
    /* Registration */
    struct icp_result {
        bool converged;
        double fitness;
        Eigen::Affine3d affine;
        int inliers;
    };

    icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int iterations=1, float max_corr=0.05);
    icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
        Eigen::Vector3f axis, float angle, Eigen::Vector3f translation_offset, int);
    icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr offset_cloud, Eigen::Vector3f axis, float angle, Eigen::Vector3f translation_offset, int iterations);


    /* Advanced Registration */
    icp_result alp_align(PointCloudT::Ptr object, PointCloudT::Ptr scene, PointCloudT::Ptr object_aligned,
        int max_iterations, int num_samples, float similarity_thresh, float max_corresp_dist, float inlier_frac);

    // icp_result sac_icp(PointCloudT::Ptr object, PointCloudT::Ptr scene);
    icp_result sac_icp(PointCloudT::Ptr object, PointCloudT::Ptr scene, Eigen::Affine3d seed_pose);



    /* Visualization */
    void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
    void visualize_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud);

    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2);
    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2, std::string window_name);
    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string window_name);

    void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string window_name);
    void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_2, std::string window_name);

    void visualize(pcl::PointCloud<pcl::PointNormal>::Ptr scene, pcl::PointCloud<pcl::PointNormal>::Ptr object);
    void visualize(PointCloudT::Ptr scene, std::string window_name);

    void color_cloud(int pcl_color, pcl::PointCloud<pcl::PointXYZ> &input_cloud, pcl::PointCloud<pcl::PointXYZRGBA> &destination_cloud);
    int pcl_color(float r, float g, float b);


    /* Transformation */
    void discard_halfspace(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
        pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud);
    void discard_halfspace(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
        pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud);

    void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud);
    void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud);

    void affine_cloud(Eigen::Affine3d transform, pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud);
    void affine_cloud(Eigen::Affine3d transform, pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud);

    int closest_point_line(pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Vector3f direction, Eigen::Vector3f line_pt);
    int closest_point_line(pcl::PointCloud<pcl::PointXYZRGBA>& cloud, Eigen::Vector3f direction, Eigen::Vector3f line_pt);


    /* Segmentation */
    void extract_indices(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, pcl::PointIndices::Ptr &cluster);
    void segment_region_growing(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, int index, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud);

}
