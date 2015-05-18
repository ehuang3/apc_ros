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
#include "pcl_tools.h"

namespace pcl_tools {
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
}