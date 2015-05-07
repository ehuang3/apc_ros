#include <pcl/point_cloud.h>
#include <stdlib.h>
#include <Eigen/Core>

typedef pcl::PointXYZRGB PointType;

float fx;
float fy;

void frustCull(pcl::PointCloud<PointType>::Ptr input_scene,pcl::PointCloud<PointType> output,float x1,float x2, float y1,float y2)
{
    pcl::FrustumCulling<pcl::PointXYZ> culler;
    bounding_center_x= x2>x1 ? x2-calcWidth(x1,x2) : x1-calcWidth(x1,x2);
    bounding_center_y= y2>y1 ? y2-calcWidth(y1,y2) : y1-calcWidth(y1,y2);
    camera_pose_x=-(fx-bounding_center_x);
    camera_pose_y=-(fy-bounding_center_y);
    Eigen::Matrix4f camera_pose;
    camera_pose << 0, 0, 0, 0
                 0, 0, 0, camera_pose_y
                 0, 0, 0, camera_pose_x
                 0, 0, 0, 1;
    culler.setInputCloud (input_scene);
    culler.setVerticalFOV (45);
    culler.setHorizontalFOV (60);
    culler.setNearPlaneDistance (.60);
    culler.setFarPlaneDistance (1.02);
    culler.setCameraPose (camera_pose);
    culler.filter(output);
}

//since it is a distance we want
float calcWidth(float value_1,float value_2)
{
    return abs(value_2-value_1);
}

float calcHeight(float value_1, float value_2)
{
    return abs(value_2-value_1);
}
