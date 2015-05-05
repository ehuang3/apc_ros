#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "transform.cpp"

struct icp_result {
    bool converged;
    double fitness;
};

icp_result apply_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int iterations=1) {
    /* Attempts to align input_cloud to target_cloud, does so in-place */
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (input_cloud);
    icp.setInputTarget (target_cloud);
    icp.align (*input_cloud);

    // icp.setMaxCorrespondence(0.05) --> Ignore correspondences beyond some maximum distance
    // icp.setTransformationEpsilon (1e-8); --> Transformation size minimum
    // icp.setEuclideanFitnessEpsilon (1); -- > change minimum

    icp_result result;
    // min: 5e-7
    result.converged = icp.hasConverged();
    result.fitness = icp.getFitnessScore();
    return result;
}

struct orientation {
    Eigen::Vector3f axis;  // Axis
    float theta;  // Angle about axis
};

// icp_result sac_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, int iterations=5) {
//     /* There are 6 key orientations and 6 key positions, for a total of 36 total poses */

//     Eigen::Vector3f origin(0.0, 0.0, 0.0);
//     std::list<Eigen::Vector3f> seed_positions(
//         Eigen::Vector3f(-1, 0, 0);
//         Eigen::Vector3f(0, -1, 0);
//         Eigen::Vector3f(0, 0, -1);
//         Eigen::Vector3f(1, 0, 0);
//         Eigen::Vector3f(0, 1, 0);
//         Eigen::Vector3f(0, 0, 1);
//     );

// //void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<PointT>& input_cloud, pcl::PointCloud<PointT>& destination_cloud)

//     for (std::list<Eigen::Vector3f>::iterator position = seed_positions.begin(); position != seed_positions.end(); position++){
//         affine_cloud(Eigen::Vector3f::UnitZ(), 0.0)
//     }

//     icp_result result;
//     // result = apply_icp(current_cloud, target_cloud, 60);  // 60 iterations

// }