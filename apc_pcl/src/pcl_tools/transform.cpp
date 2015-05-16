#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl_tools.h"

namespace pcl_tools {
void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud) {
    /* Apply a 3D affine transformation to an arbitrary pcl::PointCloud

    Rotate input_cloud by theta radians about axis, and then translate by the translation vector, placing the result in destination 
        SLOW!
    */
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = translation;
    transform.rotate(Eigen::AngleAxisf(theta, axis));
    pcl::transformPointCloud(input_cloud, destination_cloud, transform);
}

void affine_cloud(Eigen::Affine3d transform, pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud) {
    pcl::transformPointCloud(input_cloud, destination_cloud, transform);
}
void affine_cloud(Eigen::Affine3d transform, pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud) {
    pcl::transformPointCloud(input_cloud, destination_cloud, transform);
}

void discard_halfspace(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
    pcl::PointCloud<pcl::PointXYZ>& input_cloud, pcl::PointCloud<pcl::PointXYZ>& destination_cloud) {
    /* 
        Discard everything on the halfspace defined by {p | a.T * p<= 0} 
        a -> plane normal
        p -> point in cloud
        b -> plane origin (Or any point on the plane)
    */

    typename pcl::PointCloud<pcl::PointXYZ>::iterator point;
    pcl::PointXYZ new_point;

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

void affine_cloud(Eigen::Vector3f axis, float theta, Eigen::Vector3f translation, pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud) {
    /* Apply a 3D affine transformation to an arbitrary pcl::PointCloud

    Rotate input_cloud by theta radians about axis, and then translate by the translation vector, placing the result in destination 
        SLOW!
    */
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = translation;
    transform.rotate(Eigen::AngleAxisf(theta, axis));
    pcl::transformPointCloud(input_cloud, destination_cloud, transform);
}

void discard_halfspace(Eigen::Vector3f plane_normal, Eigen::Vector3f plane_origin,
    pcl::PointCloud<pcl::PointXYZRGBA>& input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>& destination_cloud) {
    /* 
        Discard everything on the halfspace defined by {p | a.T * p<= 0} 
        a -> plane normal
        p -> point in cloud
        b -> plane origin (Or any point on the plane)
    */

    typename pcl::PointCloud<pcl::PointXYZRGBA>::iterator point;
    pcl::PointXYZRGBA new_point;

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

}