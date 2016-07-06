/*
This ros node subscribes to the topics "/clm_ros_wrapper/head_position" (for headposition in camera frame)
and "/clm_ros_wrapper/head_vector" (for head fixation vector in camera frame) and computes the intersection
point of the head direction and the hardcoded screen. It then publishes this geometry_msgs::Vector3 with
publisher topic "/clm_ros_wrapper/gaze_point".
Yunus
*/
#include "CLM_core.h"

#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <clm_ros_wrapper/ClmHeads.h>
#include <clm_ros_wrapper/ClmEyeGaze.h>
#include <clm_ros_wrapper/ClmFacialActionUnit.h>
#include <clm_ros_wrapper/Scene.h>
#include <clm_ros_wrapper/Object.h>

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h> 

using namespace std;

int main(int argc, char **argv) 
{
    float screenWidth; 
    float screenHeight;
    float screenGap;

    ros::init(argc, argv, "scene_publisher");
    ros::NodeHandle nh;

    nh.getParam("screenWidth", screenWidth);
    nh.getParam("screenHeight", screenHeight);
    nh.getParam("screenGap", screenGap);

    // the dimensions of the display screen
    float display_screen_width = screenWidth - 2 * screenGap;
    float display_screen_height = screenHeight - 2 * screenGap;

    float robot_height;
    float robot_width;

    nh.getParam("robot_width", robot_width);
    nh.getParam("robot_height", robot_height);

    //publishing the current scene
    ros::Publisher scene_publisher = nh.advertise<clm_ros_wrapper::Scene>("/clm_ros_wrapper/scene", 1);

    clm_ros_wrapper::Scene current_scene;

    current_scene.num_planes = 2;

    clm_ros_wrapper::Objects planes [current_scene.num_planes];

    planes[0].num_objects = 6;

    clm_ros_wrapper::Object objects_on_plane0 [planes[0].num_objects];

    objects_on_plane0[0].name = "upper-left-point";
    objects_on_plane0[0].x_screen = display_screen_width/6;
    objects_on_plane0[0].y_screen = display_screen_height/4;

    objects_on_plane0[1].name = "upper-mid-point";
    objects_on_plane0[1].x_screen = display_screen_width/2;
    objects_on_plane0[1].y_screen = display_screen_height/4;

    objects_on_plane0[2].name = "upper-right-point";
    objects_on_plane0[2].x_screen = 5 * display_screen_width/6;
    objects_on_plane0[2].y_screen = display_screen_height/4;

    objects_on_plane0[3].name = "lower-left-point";
    objects_on_plane0[3].x_screen = display_screen_width/6;
    objects_on_plane0[3].y_screen = 3 * display_screen_height/4;

    objects_on_plane0[4].name = "lower-mid-point";
    objects_on_plane0[4].x_screen = display_screen_width/2;
    objects_on_plane0[4].y_screen = 3 * display_screen_height/4;

    objects_on_plane0[5].name = "lower-right-point";
    objects_on_plane0[5].x_screen = 5 * display_screen_width/6;
    objects_on_plane0[5].y_screen = 3 * display_screen_height/4;

    // pushing the objects back to the objects parameter of the scene message
    for (int j = 0; j < planes[0].num_objects; j++)
    {
        planes[0].objects.push_back(objects_on_plane0[j]);
    }

    planes[1].num_objects = 1;

    clm_ros_wrapper::Object objects_on_plane1 [planes[1].num_objects];

    objects_on_plane1[0].name = "robot";
    objects_on_plane1[0].x_screen = robot_height / 2;
    objects_on_plane1[0].y_screen = robot_width / 2;

    // pushing the objects back to the objects parameter of the scene message
    for (int j = 0; j < planes[1].num_objects; j++)
    {
        planes[1].objects.push_back(objects_on_plane1[j]);
    }

    while (nh.ok())
    {
        scene_publisher.publish(current_scene);
        ros::spinOnce();
    }
    
    return 0;
}
