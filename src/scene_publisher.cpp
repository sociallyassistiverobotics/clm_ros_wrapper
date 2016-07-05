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

    //publishing the current scene
    ros::Publisher scene_publisher = nh.advertise<clm_ros_wrapper::Scene>("/clm_ros_wrapper/scene", 1);

    clm_ros_wrapper::Scene current_scene;

    current_scene.num_objects = 7;

    clm_ros_wrapper::Object objects [current_scene.num_objects];

    objects[0].name = "upper-left-point";
    objects[0].x_screen = display_screen_width/6;
    objects[0].y_screen = display_screen_height/4;

    objects[1].name = "upper-mid-point";
    objects[1].x_screen = display_screen_width/2;
    objects[1].y_screen = display_screen_height/4;

    objects[2].name = "upper-right-point";
    objects[2].x_screen = 5 * display_screen_width/6;
    objects[2].y_screen = display_screen_height/4;

    objects[3].name = "lower-left-point";
    objects[3].x_screen = display_screen_width/6;
    objects[3].y_screen = 3 * display_screen_height/4;

    objects[4].name = "lower-mid-point";
    objects[4].x_screen = display_screen_width/2;
    objects[4].y_screen = 3 * display_screen_height/4;

    objects[5].name = "lower-right-point";
    objects[5].x_screen = 5 * display_screen_width/6;
    objects[5].y_screen = 3 * display_screen_height/4;

    objects[6].name = "robot";
    objects[6].x_screen = 3 * display_screen_width/2;
    objects[6].y_screen = 3 * display_screen_height/4;

    // pushing the objects back to the objects parameter of the scene message
    for (int i = 0; i < current_scene.num_objects; i++)
    {
        current_scene.objects.push_back(objects[i]);
    }


    while (nh.ok())
    {
        scene_publisher.publish(current_scene);
        ros::spinOnce();
    }
    
    return 0;
}
