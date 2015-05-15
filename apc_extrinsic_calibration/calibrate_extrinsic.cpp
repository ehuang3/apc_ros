#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>

void run(ros::ServiceClient &detect,ros::ServiceClient &calibrate)
{
    std_srvs::Empty empty_srv;
    detect.call(empty_srv);
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
    ros::spin();
    int num=3;
    while(num!=0)
    {
        num=3;
        std::cerr << "Please enter number" << std::endl;
        std::cin>> num;
        if(num==1)
        {
            run;
        }
        else if(num==2)
        {
            save;
        }
    }
    std::cerr << "ending proram" << std::endl;
}
