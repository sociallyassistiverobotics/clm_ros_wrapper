#include "TesterHelper.h"

#include <ros/ros.h>

using namespace std;

int main (int argc, char **argv)
{
    ROS_INFO("Initializing..");

    std::string name = "face_recognition_tester";
    ros::init(argc, argv, name.c_str());
    
    ros::NodeHandle nh;
    nh.getParam("ns", name);

    TesterHelper testerHelper(name, argv[0]);
    ros::spin();

    return 0;
}
