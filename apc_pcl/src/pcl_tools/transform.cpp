#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

template<typename PointT> 
void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<PointT>& input_cloud, pcl::PointCloud<PointT>& destination_cloud) {
    /* Apply a 3D affine transformation to an arbitrary pcl::PointCloud

    Rotate input_cloud by theta radians about axis, and then translate by the translation vector, placing the result in destination 
        SLOW!
    */
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = translation;
    transform.rotate(Eigen::AngleAxisf(theta, axis));
    pcl::transformPointCloud(input_cloud, destination_cloud, transform);
}

template<typename PointT>
void discard_halfspace(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
    pcl::PointCloud<PointT>& input_cloud, pcl::PointCloud<PointT>& destination_cloud) {
    /* 
        Discard everything on the halfspace defined by {p | a.T * p<= 0} 
        a -> plane normal
        p -> point in cloud
        b -> plane origin (Or any point on the plane)
    */

    typename pcl::PointCloud<PointT>::iterator point;
    PointT new_point;

    for (point = input_cloud.points.begin(); point < input_cloud.end(); point++) {
        Eigen::Vector3f v_point;
        v_point << point->x, point->y, point->z;
        Eigen::Vector3f shifted_point = v_point - plane_origin;
        float dot = plane_normal.transpose().dot(shifted_point);

        if (dot < 0) {
            new_point.x = v_point[0];
            new_point.y = v_point[1];
            new_point.z = v_point[2];
            destination_cloud.push_back(new_point);
        }
    }
}

// template<typename PointT>
// void add_plane(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
//     pcl::PointCloud<PointT>& input_cloud, pcl::PointCloud<PointT>& destination_cloud, float spacing=0.1) {
//     /* Add a plane with evenly spaced points, with "spacing" units between them */
    
// }