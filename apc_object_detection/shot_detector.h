#ifndef SHOT_DETECTOR_H
#define SHOT_DETECTOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <iostream>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT1344 DescriptorType;
typedef pcl::PFHRGBSignature250 FeatureType;
struct CloudStyle
{
    double r;
    double g;
    double b;
    double size;

    CloudStyle (double r,
                double g,
                double b,
                double size) :
        r (r),
        g (g),
        b (b),
        size (size)
    {
    }
};

CloudStyle style_white (255.0, 255.0, 255.0, 4.0);
CloudStyle style_red (255.0, 0.0, 0.0, 3.0);
CloudStyle style_green (0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan (93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet (255.0, 0.0, 255.0, 8.0);

int icp_max_iter_ (5);
float icp_corr_distance_ (0.1f);
float hv_clutter_reg_ (5.0f);
float hv_inlier_th_ (0.03f);
float hv_occlusion_th_ (0.01f);
float hv_rad_clutter_ (0.03f);
float hv_regularizer_ (3.0f);
float hv_rad_normals_ (0.05);
bool hv_detect_clutter_ (true);
class shot_detector
{
public:
    shot_detector();

    void processImage();

    void extractClusters();

private:

    // PointClouds
    pcl::PointCloud<PointType>::Ptr model;
    pcl::PointCloud<PointType>::Ptr model_keypoints;
    pcl::PointCloud<PointType>::Ptr scene;
    pcl::PointCloud<PointType>::Ptr scene_keypoints;
    pcl::PointCloud<NormalType>::Ptr model_normals;
    pcl::PointCloud<NormalType>::Ptr scene_normals;
    //pcl::PointCloud<FeatureType>::Ptr model_descriptors;
    //pcl::PointCloud<FeatureType>::Ptr scene_descriptors;
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;
    pcl::PointCloud<PointType>::Ptr model_good_kp;
    pcl::PointCloud<PointType>::Ptr scene_good_kp ;
    pcl::PointCloud<PointType>::Ptr objects ;


    //Processing functions
    void findCorrespondences (typename pcl::PointCloud<DescriptorType>::Ptr source,
                              typename pcl::PointCloud<DescriptorType>::Ptr target, std::vector<int>& correspondences);
    void filterCorrespondences ();
    void filter(pcl::PointCloud<PointType>::Ptr scene,pcl::PointCloud<PointType>::Ptr filtered_scene);

    void calcNormals(pcl::PointCloud<PointType>::Ptr cloud,pcl::PointCloud<NormalType>::Ptr normals);
    void calcSHOTDescriptors(pcl::PointCloud<PointType>::Ptr cloud,pcl::PointCloud<PointType>::Ptr keypoints
                             ,pcl::PointCloud<NormalType>::Ptr normals,pcl::PointCloud<DescriptorType>::Ptr descriptors);
    void calcPFHRGBDescriptors(pcl::PointCloud<PointType>::Ptr cloud,pcl::PointCloud<PointType>::Ptr keypoints
                               ,pcl::PointCloud<NormalType>::Ptr normals,pcl::PointCloud<FeatureType>::Ptr descriptors);

    void downsample(pcl::PointCloud<PointType>::Ptr cloud,pcl::PointCloud<PointType>::Ptr keypoints, float sample_size);
    void compare(pcl::PointCloud<FeatureType>::Ptr model_descriptions,
                 pcl::PointCloud<FeatureType>::Ptr scene_descriptions);
    void houghGrouping();
    void output();
    void outwithver();

    void voxelFilter(pcl::PointCloud<PointType>::Ptr cloud,pcl::PointCloud<PointType>::Ptr filtered_cloud, float sample_size);

    void planarExtraction(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr clustered_objects);
    // Detection thresholds
    float model_ss_;
    float scene_ss_;
    float rf_rad_ ;
    float descr_rad_;
    float cg_size_ ;
    float cg_thresh_;
    float corr_dist_;
    float voxel_sample_;

    //PointCloud classes for calculating various features
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    pcl::UniformSampling<PointType> uniform_sampling;
    pcl::SHOTColorEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    pcl::PFHRGBEstimation<PointType, NormalType, FeatureType> pfhrgbEstimation;
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    pcl::VoxelGrid<PointType> voxel_filter;

    //PointCloud data
    pcl::CorrespondencesPtr model_scene_corrs;
    std::vector<int> model_good_keypoints_indices;
    std::vector<int> scene_good_keypoints_indices;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;
    std::vector<int> source2target_;
    std::vector<int> target2source_;
    pcl::CorrespondencesPtr correspondences_;

    //ROS stuff
    bool data;
    ros::NodeHandle nh;
    ros::Subscriber kinect;
    sensor_msgs::PointCloud2 depth_msg;
    void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &data_msg);
    void convertMsg(sensor_msgs::PointCloud2 data_msg,pcl::PointCloud<PointType>::Ptr cloud);

    // Helper functions
    double computeCloudResolution(const pcl::PointCloud<PointType>::Ptr cloud);
    void loadParameters();
public:
    bool activated;
};

#endif // SHOT_DETECTOR_H

