#include "InitializerHelper.h"

#include <ros/ros.h>

using namespace std;

int main (int argc, char **argv)
{
    ROS_INFO("Initializing..");

    std::string name = "face_recognition_initializer";
    ros::init(argc, argv, name.c_str());
    
    ros::NodeHandle nh;
    nh.getParam("ns", name);

    InitializerHelper initializerHelper(name, argv[0]);
    ros::spin();

    return 0;
}
