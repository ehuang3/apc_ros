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
#include <math.h>
#include "../pcl_tools/pcl_tools.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudxyz;

int closest_point_line(PointCloudxyz& cloud, Eigen::Vector3f direction, Eigen::Vector3f line_pt) {

    double min_distance = 10;
    int min_index = 0;

    // int index = 0;

    pcl::PointXYZ point;
    // typename pcl::PointCloud<pcl::PointXYZ>::iterator point;
    // for (point = cloud.points.begin(); point < cloud.points.end(); point++) {
    for (int index = 0; index < cloud.points.size(); index++) {
        point = cloud.points[index];

        Eigen::Vector3f cloud_pt(point.x, point.y, point.z);
        double projection = (cloud_pt - line_pt).dot(direction);
        double hypotenuseSq = (cloud_pt - line_pt).squaredNorm();
        double distance = sqrt(hypotenuseSq - pow(projection, 2));

        if (distance < min_distance) {
            pcl::console::print_highlight ("FINDO%f\n", distance);

            min_distance = distance;
            min_index = index;
        }
    }
    return(min_index);
}

int main (int argc, char **argv) {
    PointCloudxyz::Ptr object (new PointCloudxyz);
    PointCloudxyz::Ptr scene (new PointCloudxyz);
    PointCloudxyz::Ptr object_aligned (new PointCloudxyz);

    // Get input object and scene
    if (argc != 2)
    {
        pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
        return (1);
    }

    // Load object and scene
    pcl::console::print_highlight ("Loading point clouds...\n");
    pcl_tools::cloud_from_ply(argv[1], *object);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*object, *object, indices);

    int index = closest_point_line(*object, Eigen::Vector3f(0.57735054, 0.57735054, 0.57735054), Eigen::Vector3f(0.0, 0.0, 0.0));
    // int index = closest_point_line(*object, Eigen::Vector3f::UnitY(), Eigen::Vector3f(0.0, 0.0, 0.0));


    pcl::PointXYZ centerpt = object->points[index];


    pcl::visualization::PCLVisualizer visu("LineFind");
    visu.addPointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 0.0, 255.0), "object");
    visu.addSphere(centerpt, 0.005, "location");
    // visu.addLine<Eigen::Vector4f, Eigen::Vector4f>(Eigen::Vector4f(0.0, 0.0, 0.0, 1.0), Eigen::Vector4f(0.0, 0.0, 0.0, 1.0) + Eigen::Vector4f::UnitX(), "line");
    visu.addCoordinateSystem (0.04, "View_1");
    // visu.addPointCloudNormals<PointNT>(object);

    visu.spin ();


}