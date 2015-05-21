#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "pcl_tools.h"
// #include "transform.h"
namespace pcl_tools {
icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int iterations, float max_corr) {
    /* Attempts to align input_cloud to target_cloud, does so in-place */
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setMaxCorrespondenceDistance(max_corr);
    icp.setTransformationEpsilon (0.2);
    icp.setInputSource (input_cloud);
    icp.setInputTarget (target_cloud);
    icp.align (*input_cloud);

    // icp.setMaxCorrespondence(0.05) --> Ignore correspondences beyond some maximum distance
    // icp.setTransformationEpsilon (1e-8); --> Transformation size maximum
    // icp.setEuclideanFitnessEpsilon (1); -- > change minimum

    icp_result result;
    // min: 5e-7
    result.converged = icp.hasConverged();
    result.fitness = icp.getFitnessScore();
    result.affine = Eigen::Affine3d(icp.getFinalTransformation().cast<double>());
    std::cout << icp.getFinalTransformation() << std::endl;
    std::cout << "Fitness: " << icp.getFitnessScore() << std::endl;
    return result;
}

icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
    Eigen::Vector3f axis, float angle, Eigen::Vector3f translation_offset, int iterations=1) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr offset_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    affine_cloud(axis, angle, translation_offset, *input_cloud, *offset_cloud);

    icp_result result;
    result = apply_icp(offset_cloud, target_cloud, iterations);

    return result;
}

icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr offset_cloud, Eigen::Vector3f axis, float angle, Eigen::Vector3f translation_offset, int iterations=1) {

    // pcl::PointCloud<pcl::PointXYZ>::Ptr offset_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    affine_cloud(axis, angle, translation_offset, *input_cloud, *offset_cloud);

    *offset_cloud = *input_cloud;
    icp_result result;
    result = apply_icp(offset_cloud, target_cloud, iterations);

    return result;
}

struct orientation {
    Eigen::Vector3f axis;  // Axis
    float theta;  // Angle about axis
};

// icp_result sac_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int iterations=5) {
//     /* There are 6 key orientations and 6 key positions, for a total of 36 total poses */

//     Eigen::Vector3f origin(0.0, 0.0, 0.0);
//     std::list<Eigen::Vector3f> seed_positions{
//         Eigen::Vector3f(-1, 0, 0),
//         Eigen::Vector3f(0, -1, 0),
//         Eigen::Vector3f(0, 0, -1),
//         Eigen::Vector3f(1, 0, 0),
//         Eigen::Vector3f(0, 1, 0),
//         Eigen::Vector3f(0, 0, 1)
//     };

//     for (std::list<Eigen::Vector3f>::iterator position = seed_positions.begin(); position != seed_positions.end(); position++){
//         std::cout << "testing position " << *position << std::endl;
//         apply_icp(input_cloud, target_cloud, Eigen::Vector3f::UnitZ(), 0.0, *position);
//         // affine_cloud(Eigen::Vector3f::UnitZ(), 0.0, *position);
//     }

//     icp_result result;
//     // result = apply_icp(current_cloud, target_cloud, 60);  // 60 iterations

// }

}