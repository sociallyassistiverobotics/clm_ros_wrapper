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
#include <clm_ros_wrapper/FreeObject.h>

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

    //SCREEN

    //number of objects on the screen
    current_scene.screen.num_objects_on_screen = 6;

    // constructing the array to define the objects on the screen
    clm_ros_wrapper::Object objects_on_screen [current_scene.screen.num_objects_on_screen];

    objects_on_screen[0].name = "upper-left-point";
    objects_on_screen[0].x_screen = display_screen_width/6;
    objects_on_screen[0].y_screen = display_screen_height/4;

    objects_on_screen[1].name = "upper-mid-point";
    objects_on_screen[1].x_screen = display_screen_width/2;
    objects_on_screen[1].y_screen = display_screen_height/4;

    objects_on_screen[2].name = "upper-right-point";
    objects_on_screen[2].x_screen = 5 * display_screen_width/6;
    objects_on_screen[2].y_screen = display_screen_height/4;

    objects_on_screen[3].name = "lower-left-point";
    objects_on_screen[3].x_screen = display_screen_width/6;
    objects_on_screen[3].y_screen = 3 * display_screen_height/4;

    objects_on_screen[4].name = "lower-mid-point";
    objects_on_screen[4].x_screen = display_screen_width/2;
    objects_on_screen[4].y_screen = 3 * display_screen_height/4;

    objects_on_screen[5].name = "lower-right-point";
    objects_on_screen[5].x_screen = 5 * display_screen_width/6;
    objects_on_screen[5].y_screen = 3 * display_screen_height/4;

    // pushing the objects back to the objects_on_screen component of current_scene.screen
    for (int i = 0; i < current_scene.screen.num_objects_on_screen; i++)
    {
        current_scene.screen.objects_on_screen.push_back(objects_on_screen[i]);
    }

    //FREE OBJECTS

    current_scene.num_free_objects = 1;

    clm_ros_wrapper::FreeObject free_objects [current_scene.num_free_objects];

    free_objects[0].name = "robot";
    
    // Reading robot position from the parameter server
    float robot_position_wf_x, robot_position_wf_y, robot_position_wf_z;
    nh.getParam("robot_position_wf_x", robot_position_wf_x);
    nh.getParam("robot_position_wf_y", robot_position_wf_y);
    nh.getParam("robot_position_wf_z", robot_position_wf_z);

    tf::Vector3 position_tf = tf::Vector3(robot_position_wf_x, robot_position_wf_y, robot_position_wf_z);

    geometry_msgs::Vector3 position_msg;

    tf::vector3TFToMsg(position_tf, position_msg);

    free_objects[0].position = position_msg;

    // pushing the free objects back to the free_objects component of the scene message
    for (int i = 0; i < current_scene.num_free_objects; i++)
    {
        current_scene.free_objects.push_back(free_objects[i]);
    }

    while (nh.ok())
    {
        scene_publisher.publish(current_scene);
        ros::spinOnce();
    }
    
    return 0;
}
