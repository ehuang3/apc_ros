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
#include <pcl/registration/icp_nl.h>
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

bool in_icp = false;
float max_corr_icp = 0.01;
float max_eps_icp = 0.2;

void compute_features(PointCloudT::Ptr cloud, FeatureCloudT::Ptr feature_cloud) {
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (feature_radius);
  fest.setInputCloud (cloud);
  fest.setInputNormals (cloud);
  fest.compute (*feature_cloud);
}

void compute_normals(PointCloudT::Ptr cloud) {
  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (normal_radius);
  nest.setInputCloud (cloud);
  nest.compute (*cloud);

  pcl::visualization::PCLVisualizer visu("Alignment");
  visu.addPointCloudNormals<PointNT>(cloud, 1);
  visu.spin();
}

pcl_tools::icp_result alp_align(PointCloudT::Ptr object, PointCloudT::Ptr scene, PointCloudT::Ptr object_aligned,
    int max_iterations, int num_samples, float similarity_thresh, float max_corresp_dist, float inlier_frac, float leaf) {
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  compute_normals(object);
  compute_normals(scene);

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  // const float leaf = 0.005f;
  // const float leaf = in_leaf;

  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);

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
  align.setCorrespondenceRandomness (12); // Number of nearest features to use
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

  pcl::console::parse_argument (argc, argv, "--icp", in_icp);
  pcl::console::parse_argument (argc, argv, "--max_corr_icp", max_corr_icp);
  pcl::console::parse_argument (argc, argv, "--icp_eps", max_eps_icp);

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

  // /*pcl_tools::icp_result align = */alp_align(object, scene, object_aligned, 50000, 3, 0.9f, 5.5f * leaf, 0.7f);
  // /*pcl_tools::icp_result align = */alp_align(object_aligned, scene, object_aligned, 50000, 3, 0.9f, 7.5f * leaf, 0.4f);

  std::cout << "Inlier frac " << in_inlier_frac << std::endl;
  pcl_tools::icp_result align = alp_align(object, scene, object_aligned, in_max_iterations, in_num_samples, in_similarity_thresh, in_max_corresp_dist, in_inlier_frac, in_leaf);

  pcl::visualization::PCLVisualizer visu("Alignment");
  if (align.converged)
  {
    pcl::console::print_info ("Inliers: %i/%i, scene: %i\n", align.inliers, object->size (), scene->size ());
    
    // Show alignment
    visu.addPointCloud (object, ColorHandlerT (object, 255.0, 0.0, 0.0), "object");
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

  if (in_icp) {
    pcl::console::print_highlight ("Applying ICP now\n");
    pcl::IterativeClosestPointNonLinear<PointNT, PointNT> icp;
    // pcl::IterativeClosestPoint<PointNT, PointNT> icp;
    pcl_tools::affine_cloud(Eigen::Vector3f::UnitZ(), 0.0, Eigen::Vector3f(0.0, 0.0, 0.02), *object_aligned, *object_aligned);

    icp.setMaximumIterations (100);
    icp.setMaxCorrespondenceDistance(max_corr_icp);
    icp.setTransformationEpsilon (max_eps_icp);
    icp.setInputSource (object_aligned);
    icp.setInputTarget (scene);
    icp.align (*object_aligned);

    if (icp.hasConverged()) {
      pcl::console::print_highlight ("ICP: Converged with fitness %f\n", icp.getFitnessScore());
    }
    // pcl::visualization::PCLVisualizer visu("Alignment");
    // visu.addPointCloud (object, ColorHandlerT (object, 255.0, 0.0, 0.0), "object");
    // visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.updatePointCloud (object_aligned, ColorHandlerT (object_aligned, 100.0, 50.0, 200.0), "object_aligned");

    // visu.addPointCloudNormals<PointNT>(object);

    visu.spin ();

  }
  return (0);
}

/* Settings that worked well



(Works well for Genuine Joe) -- And Crayola
--leaf 0.01 --inlier_frac 0.7 --max_cdist 0.05 --normal_radius 0.01 --feature_radius 0.02 --max_iterations 50000
--leaf 0.01 --inlier_frac 0.7 --max_cdist 0.04 --normal_radius 0.01 --feature_radius 0.02 --max_iterations 500000
--leaf 0.01 --inlier_frac 0.5 --max_cdist 0.02 --normal_radius 0.01 --feature_radius 0.02 --max_iterations 500000


- Oreo box suffers because of all of the missing points
- Kitty litter is easy (why? Lots of differently oriented normals)


Ridiculous downsampling
--leaf 0.05 --inlier_frac 0.25 --max_cdist 0.01 --normal_radius 0.01 --feature_radius 0.02 --max_iterations 500000 --icp 1 --max_corr_icp 0.02 --max_eps_icp 0.000001

*/