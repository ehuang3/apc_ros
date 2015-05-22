#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "../pcl_tools/pcl_tools.h"


typedef pcl::PointXYZ InPointType;

int main (int argc, char** argv)
{
    typedef pcl::PointXYZ cloud_type;

    pcl::console::TicToc time;

    PointCloudT::Ptr object(new PointCloudT);
    PointCloudT::Ptr scene(new PointCloudT);
    PointCloudT::Ptr object_aligned(new PointCloudT);

    if (argc != 3)
    {
        pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
        return (1);
    }

    // Load object and scene
    pcl::console::print_highlight ("Loading point clouds...\n");

    pcl_tools::cloud_from_stl(argv[2], *object);

    if (pcl::io::loadPCDFile<PointNT> (argv[1], *scene) < 0)
    {
        pcl::console::print_error ("Error loading object/scene file!\n");
        return (1);
    }

    // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*scene, *scene, indices);
    pcl::removeNaNFromPointCloud(*object, *object, indices);

    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch (0.04);
    nest.setInputCloud (scene);
    // nest.setKSearch (10);
    nest.compute (*scene);
    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloud<PointNT>(scene, "scenenee");
    viewer.addPointCloudNormals<PointNT>(scene, 10);

    while (!viewer.wasStopped ())
    {
    viewer.spinOnce ();
    }



    pcl::console::print_highlight ("Downsampling for registration\n");
    pcl::VoxelGrid<pcl::PointNormal> grid;
    const float leaf = 0.005f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);
    grid.setInputCloud (scene);
    grid.filter (*scene);

    // alp_align(PointCloudT::Ptr object, PointCloudT::Ptr scene, PointCloudT::Ptr object_aligned,
        // int max_iterations, int num_samples, float similarity_thresh, float max_corresp_dist, float inlier_frac)

    pcl_tools::icp_result result1 = pcl_tools::alp_align(object, scene, object_aligned, 50000, 3, 0.9f, 5.5f * leaf, 0.7f);
    pcl_tools::visualize(scene, object_aligned);

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << result1.affine.matrix().format(CleanFmt) << std::endl;
    pcl_tools::icp_result result = pcl_tools::sac_icp(object, scene, result1.affine);

    return(0);
}