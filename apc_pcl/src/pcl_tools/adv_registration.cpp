#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#define _USE_MATH_DEFINES
#include <cmath> 

#include "pcl_tools.h"

namespace pcl_tools {

bool next_iteration = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    pcl::console::print_highlight ("Got a keyboard event\n");
    next_iteration = true;
}

void compute_features(PointCloudT::Ptr cloud, FeatureCloudT::Ptr feature_cloud) {
  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.001);
  nest.setInputCloud (cloud);
  nest.compute (*cloud);
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (cloud);
  fest.setInputNormals (cloud);
  fest.compute (*feature_cloud);
}

icp_result alp_align(PointCloudT::Ptr object, PointCloudT::Ptr scene, PointCloudT::Ptr object_aligned,
    int max_iterations, int num_samples, float similarity_thresh, float max_corresp_dist, float inlier_frac) {

  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  compute_features(object, object_features);
  compute_features(scene, scene_features);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (max_iterations); // Number of RANSAC iterations
  align.setNumberOfSamples (num_samples); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (similarity_thresh); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (max_corresp_dist); // Inlier threshold
  align.setInlierFraction (inlier_frac); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  pcl_tools::icp_result result;
  result.affine = Eigen::Affine3d(align.getFinalTransformation().cast<double>());
  result.converged = align.hasConverged();
  result.inliers = align.getInliers ().size ();
  return result;
}

icp_result normals_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int iterations, float max_corr) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setMaxCorrespondenceDistance(max_corr);
    icp.setTransformationEpsilon (0.2);
    icp.setInputSource (input_cloud);
    icp.setInputTarget (target_cloud);
    icp.align (*input_cloud);

    icp_result result;
    result.converged = icp.hasConverged();
    result.fitness = icp.getFitnessScore();
    result.affine = Eigen::Affine3d(icp.getFinalTransformation().cast<double>());
}

struct orientation {
  Eigen::Vector3d axis;
  float theta;
};

icp_result sac_icp(PointCloudT::Ptr object, PointCloudT::Ptr scene, Eigen::Affine3d seed_pose) {
  /* Apply ICP at many different offsets and orientations to seek a more accurate pose estimate */
  // This is good, allow configurable distance-offset
  Eigen::Vector3d offsets[] = {
    Eigen::Vector3d(0., 0., 0. ),
    Eigen::Vector3d(-0.06, 0, 0),
    Eigen::Vector3d(0, -0.06, 0),
    Eigen::Vector3d(0, 0, -0.06),
    Eigen::Vector3d(0.06, 0, 0 ),
    Eigen::Vector3d(0, 0.06, 0 ),
    Eigen::Vector3d(0, 0, 0.06 )
  };

  // -> Do this better (try 45 deg increments, try auto-generating)
  orientation orientations[] = {
    {Eigen::Vector3d::UnitZ(), 0.0},
    {Eigen::Vector3d::UnitZ(), M_PI / 5},
    {Eigen::Vector3d::UnitZ(), 2 * M_PI / 5},
    {Eigen::Vector3d::UnitZ(), 3 * M_PI / 5},
    {Eigen::Vector3d::UnitZ(), 4 * M_PI / 5},
    {Eigen::Vector3d::UnitZ(), M_PI},

    {Eigen::Vector3d::UnitX(), M_PI / 5},
    {Eigen::Vector3d::UnitX(), 2 * M_PI / 5},
    {Eigen::Vector3d::UnitX(), 3 * M_PI / 5},
    {Eigen::Vector3d::UnitX(), 4 * M_PI / 5},
    {Eigen::Vector3d::UnitX(), M_PI},

    {Eigen::Vector3d::UnitY(), M_PI / 5},
    {Eigen::Vector3d::UnitY(), 2 * M_PI / 5},
    {Eigen::Vector3d::UnitY(), 3 * M_PI / 5},
    {Eigen::Vector3d::UnitY(), 4 * M_PI / 5},
    {Eigen::Vector3d::UnitY(), M_PI},
  };

  // void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<pcl::PointXYZ>& input_cloud, 
  // pcl::PointCloud<pcl::PointXYZ>& destination_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*scene, *xyz_scene); // Try this with pointNormals (Can we do this?)

  pcl::visualization::PCLVisualizer viewer ("Point Cloud Visualization ENHANCED!");
  viewer.setSize (1280, 1024);  // Visualiser window size
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color_h(xyz_scene, 255, 0, 0);

  viewer.addPointCloud(xyz_scene, scene_color_h, "scene");
  pcl::PointCloud<pcl::PointXYZ>::Ptr affined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  viewer.addPointCloud(affined_cloud, "object");

  // Should be using .reset() instead of new, right? I duno
  // Eigen::Affine3d initial_transform;
  // Eigen::Affine3d transform;
  Eigen::Matrix3d seed_rotation = seed_pose.linear();
  Eigen::Vector3d seed_translation = seed_pose.translation();

  for (int k = 0; k < 7; k++) {
    for (int j = 0; j < 16; j++) {
      pcl::console::print_highlight ("At %i %i\n", k, j);

      affined_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*object, *affined_cloud);

      Eigen::Matrix3d rotation = seed_rotation * (Eigen::AngleAxisd(orientations[j].theta, orientations[j].axis));
      Eigen::Vector3d translation = seed_translation + offsets[k];

      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      transform.rotate(rotation);
      transform.translation() = translation;
      pcl::transformPointCloud(*affined_cloud, *affined_cloud, transform);

      // affine_cloud(orientations[j].axis, orientations[j].theta, offsets[k], *affined_cloud, *affined_cloud);
      /*icp_result result = */apply_icp(affined_cloud, xyz_scene, 20, 0.01);
      // /*icp_result result = */apply_icp(affined_cloud, xyz_scene, 40, 0.01);
      // icp_result result = apply_icp(affined_cloud, xyz_scene, 40, 0.005);


      // affine_cloud(result.affine, *affined_cloud, *affined_cloud);

      viewer.updatePointCloud(affined_cloud, "object");
      viewer.updatePointCloud(xyz_scene, scene_color_h, "scene");
      next_iteration = false;

      // visualize(affined_cloud, xyz_scene, "Post-icp");
      while(next_iteration == false) {
        viewer.spinOnce();
      };
    }
  }
}

}