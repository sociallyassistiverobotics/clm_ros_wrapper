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

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h> 

using namespace std;
using namespace cv;

using namespace boost::filesystem;

int main(int argc, char **argv) 
{
   float screenWidth; 
   float screenHeight;
   float screenGap;

   nh.getParam("region_detecter/screenWidth", screenWidth);
   nh.getParam("region_detecter/screenHeight", screenHeight);
   nh.getParam("region_detecter/screenGap", screenGap);

   float display_screen_width = screenWidth - 2 * screenGap;
   float display_screen_height = screenHeight - 2 * screenGap;

	ros::init(argc, argv, "find_gazepoint");
	ros::NodeHandle nh;

	ros::Publisher scene_publisher = nh.advertise<clm_ros_wrapper::Scene>("clm_ros_wrapper/scene", 1);

	clm_ros_wrapper::Scene current_scene;

   current_scene.num_object = 6;

   current_scene.scene[0].name = "upper-left-point"
   current_scene.scene[0].x_scene = display_screen_width/6;
   current_scene.scene[0].y_scene = display_screen_height/4;

   current_scene.scene[0].name = "upper-mid-point"
   current_scene.scene[0].x_scene = display_screen_width/2;
   current_scene.scene[0].y_scene = display_screen_height/4;

   current_scene.scene[0].name = "upper-right-point"
   current_scene.scene[0].x_scene = 5 * display_screen_width/6;
   current_scene.scene[0].y_scene = display_screen_height/4;

   current_scene.scene[0].name = "lower-left-point"
   current_scene.scene[0].x_scene = display_screen_width/6;
   current_scene.scene[0].y_scene = 3 * display_screen_height/4;

   current_scene.scene[0].name = "lower-mid-point"
   current_scene.scene[0].x_scene = display_screen_width/2;
   current_scene.scene[0].y_scene = 3 * display_screen_height/4;

   current_scene.scene[0].name = "lower-right-point"
   current_scene.scene[0].x_scene = 5 * display_screen_width/6;
   current_scene.scene[0].y_scene = 3 * display_screen_height/4;

   scene_publisher.publish(current_scene);

   // screen_reference_points[4] = tf::Vector3((-5)* screenWidth / 12, screenHeight * cos(screenAngle) / 4, screenHeight * sin(screenAngle) / 4);
   // screen_reference_points[1] = tf::Vector3((-5) * screenWidth / 12, (3) * screenHeight * cos(screenAngle) / 4, 3 * screenHeight * sin(screenAngle) / 4);
   // screen_reference_points[6] = tf::Vector3(screenWidth * 5/12, (1) * screenHeight * cos(screenAngle) / 4, (1) * screenHeight * sin(screenAngle) / 4);
   // screen_reference_points[3] = tf::Vector3(screenWidth * 5/12, (3) * screenHeight * cos(screenAngle) / 4, (3) * screenHeight * sin(screenAngle) / 4);
   // screen_reference_points[2] = (screen_reference_points[1]+screen_reference_points[3])/2;
   // screen_reference_points[5] = (screen_reference_points[4]+screen_reference_points[6])/2;

	ros::spin();
}
