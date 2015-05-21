#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../pcl_tools/pcl_tools.h"

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

int in_max_iterations = 50000;
int in_num_samples = 3;
float in_similarity_thresh = 0.9f;
float in_max_corresp_dist = 5.5f;
float in_inlier_frac = 0.7f;
float in_leaf = 0.005f;

float feature_radius = 0.025;
float normal_radius = 0.001;


void compute_features(PointCloudT::Ptr cloud, FeatureCloudT::Ptr feature_cloud) {
  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (normal_radius);
  nest.setInputCloud (cloud);
  nest.compute (*cloud);
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (feature_radius);
  fest.setInputCloud (cloud);
  fest.setInputNormals (cloud);
  fest.compute (*feature_cloud);
}

pcl_tools::icp_result alp_align(PointCloudT::Ptr object, PointCloudT::Ptr scene, PointCloudT::Ptr object_aligned,
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

// Align a rigid object to a scene with clutter and occlusions
int main (int argc, char **argv)
{

  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);

  // Get input object and scene
  if (argc < 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }
  pcl::console::parse_argument (argc, argv, "--max_iterations", in_max_iterations);
  pcl::console::parse_argument (argc, argv, "--num_samples", in_num_samples);
  pcl::console::parse_argument (argc, argv, "--s_thresh", in_similarity_thresh);
  pcl::console::parse_argument (argc, argv, "--max_cdist", in_max_corresp_dist);
  pcl::console::parse_argument (argc, argv, "--inlier_frac", in_inlier_frac);
  pcl::console::parse_argument (argc, argv, "--leaf", in_leaf);
  pcl::console::parse_argument (argc, argv, "--normal_radius", normal_radius);
  pcl::console::parse_argument (argc, argv, "--feature_radius", feature_radius);


  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");

  pcl_tools::cloud_from_stl(argv[2], *object);

  if (pcl::io::loadPCDFile<PointNT> (argv[1], *scene) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*scene, *scene, indices);
  pcl::removeNaNFromPointCloud(*object, *object, indices);

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  // const float leaf = 0.005f;
  const float leaf = in_leaf;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);

  // /*pcl_tools::icp_result align = */alp_align(object, scene, object_aligned, 50000, 3, 0.9f, 5.5f * leaf, 0.7f);
  // /*pcl_tools::icp_result align = */alp_align(object_aligned, scene, object_aligned, 50000, 3, 0.9f, 7.5f * leaf, 0.4f);

  std::cout << "Inlier frac " << in_inlier_frac << std::endl;
  pcl_tools::icp_result align = alp_align(object, scene, object_aligned, in_max_iterations, in_num_samples, in_similarity_thresh, in_max_corresp_dist, in_inlier_frac);

  if (align.converged)
  {
    pcl::console::print_info ("Inliers: %i/%i\n", align.inliers, object->size ());
    
    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    // visu.addPointCloudNormals<PointNT>(object);

    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }
  

  return (0);
}