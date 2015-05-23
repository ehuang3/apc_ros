#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>

void detect(ros::ServiceClient &detect)
{
    std_srvs::Empty empty_srv;
    detect.call(empty_srv);
}

void calib(ros::ServiceClient &calibrate)
{
    std_srvs::Empty empty_srv;
    calibrate.call(empty_srv);
}

void save(ros::ServiceClient &save){
    std_srvs::Empty empty_srv;
    save.call(empty_srv);
}

int main( int argc,char** argv)
{
    ros::init (argc, argv, "apc_object_detection");
    ros::NodeHandle nh;
    std_srvs::Empty empty_srv;
    ros::ServiceClient detect_srv = nh.serviceClient<std_srvs::Empty>("/excel_calib_srv/detect_targets");
    ros::ServiceClient calibrate_srv = nh.serviceClient<std_srvs::Empty>("/excel_calib_srv/calibrate");
    ros::ServiceClient save_srv = nh.serviceClient<std_srvs::Empty>("/excel_calib_srv/save");
ros::spinOnce();
std::cerr << "HI please 2 to calibrate" << std::endl;

    int num=4;
    while(num!=0)
    {
        num=4;
        std::cerr << "Please enter number" << std::endl;
        std::cin>> num;
        if(num==1)
        {
            detect(detect_srv);
        }
        else if(num==2)
        {
            calib(calibrate_srv);
        }
        else if(num==3)
        {
            save(save_srv);
        }
    }
    std::cerr << "ending proram" << std::endl;
}
