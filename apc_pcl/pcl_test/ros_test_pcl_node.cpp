#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

typedef pcl::PointXYZRGB PointType;

sensor_msgs::PointCloud2ConstPtr cloud_data;
cv::Mat image;
bool msg_flag;
void PointCloudcallback(const sensor_msgs::PointCloud2ConstPtr &data_msg)
{
    cloud_data=data_msg;
}

void Imagecallback(const sensor_msgs::ImageConstPtr &image_msg)
{
    std::cerr << "callback" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(image_msg);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
    image=cv_ptr->image;
    std::cerr << image.rows << std::endl;
    if(image.rows>0)
        msg_flag=true;
}

void convertMsg(sensor_msgs::PointCloud2 data_msg, pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(data_msg, pcl_pc);
    std::cerr << "convert" << std::endl;
    pcl::PointCloud<PointType>::Ptr image_cloud(new pcl::PointCloud<PointType> ());
    pcl::fromPCLPointCloud2(pcl_pc, *image_cloud);
    cloud=pcl::PointCloud<PointType>::Ptr (image_cloud);
}

int main (int argc, char** argv)
{
    msg_flag=false;
    // Initialize ROS
    std::cerr <<"Start" << std::endl;
    ros::init (argc, argv, "test_pcl");
    ros::NodeHandle nh("testing");
    ros::Subscriber image_sub=nh.subscribe("/camera/depth/cloud_image", 1, &Imagecallback);
    ros::Rate r(10);
    ros::spinOnce();
    int i=0;
    while(msg_flag==false && i<100)
    {
        ros::spinOnce();
        i++;
        r.sleep();
    }
    Point2f p1(300,300);
    circle(image, p1, 30, Scalar( 0, 255, 255 ), 1, 8, 0);
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);

    waitKey(0);
    return 0;
}
