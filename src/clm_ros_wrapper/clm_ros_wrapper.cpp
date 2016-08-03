///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, University of Cambridge,
// all rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "CLMWrapper.h"

#include <ros/ros.h>

using namespace std;

int main (int argc, char **argv)
{
    ROS_INFO("Initializing..");

    std::string name = "clm_ros_wrapper";
    ros::init(argc, argv, name.c_str());
    
    ros::NodeHandle nh;
    nh.getParam("ns", name);

    ClmWrapper clm_wrapper(name, argv[0]);
    ros::spin();

    return 0;
}
