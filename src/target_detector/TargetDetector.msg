
/*
This ros node subscribes to "/clm_ros_wrapper/gaze_point" and finds the target on the screen where this 
gaze point falls into. It then publishes this information with publisher topic "/clm_ros_wrapper/detect_target"
*/
#include "CLM_core.h"
#include <std_msgs/String.h>

#include <fstream>
#include <sstream>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

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
#include <clm_ros_wrapper/DetectedTarget.h>

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h> 
#include <limits>

#define GAZE_ERROR 50
#define max_num_objects 20

int screenAngleInDegrees; 
float screenWidth; 
float screenHeight;
float screenAngle;
float screenGap;


int num_objects;

tf::Vector3 screen_reference_points_wf [max_num_objects];

std::string screen_reference_points_names [max_num_objects];
ros::Publisher target_publisher;

using namespace std;
// using namespace cv;

// using namespace boost::filesystem;



void gazepoint_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{  
   // for (int i = 0; i < 6; i++)
   // cout << endl << screen_reference_points_names[i] << "\t" << screen_reference_points_wf[i].getX() << "\t" << screen_reference_points_wf[i].getY() << "\t" <<  screen_reference_points_wf[i].getZ() << endl;
   if (num_objects != 0)
   {
      tf::Vector3 gazepoint;

   	tf::vector3MsgToTF(*msg, gazepoint);
      

   	// screen_reference_points[4] = tf::Vector3((-5)* screenWidth / 12, screenHeight * cos(screenAngle) / 4, screenHeight * sin(screenAngle) / 4);
    //   screen_reference_points[1] = tf::Vector3((-5) * screenWidth / 12, (3) * screenHeight * cos(screenAngle) / 4, 3 * screenHeight * sin(screenAngle) / 4);
    //   screen_reference_points[6] = tf::Vector3(screenWidth * 5/12, (1) * screenHeight * cos(screenAngle) / 4, (1) * screenHeight * sin(screenAngle) / 4);
    //   screen_reference_points[3] = tf::Vector3(screenWidth * 5/12, (3) * screenHeight * cos(screenAngle) / 4, (3) * screenHeight * sin(screenAngle) / 4);
    //   screen_reference_points[2] = (screen_reference_points[1]+screen_reference_points[3])/2;
    //   screen_reference_points[5] = (screen_reference_points[4]+screen_reference_points[6])/2;

      int num_closest_target = 0;

      float closest_distance = std::numeric_limits<double>::max();

      // do a check to see if the point is inside
      // if (closest_distance > 175) // define 175 better 
      // {
      // 	num_closest_target = 0;
      // }


      for (int i = 0; i<num_objects; i++)
      {
      	//cout<< i << "    "<< screen_reference_points[i].getX() << " " << screen_reference_points[i].getY() << "    " << screen_reference_points[i].getZ() << endl;
      	if (closest_distance > gazepoint.distance(screen_reference_points_wf[i]))
      	{
      		closest_distance = gazepoint.distance(screen_reference_points_wf[i]);
      		num_closest_target = i;
      	}
      }

      if (screen_reference_points_names[num_closest_target].compare("robot")!=0)
      {
         // WHAT IF THE POINT IS OUTSIDE THE SCREEN
         // do a check to see if the point is inside
         if ((-1)* GAZE_ERROR > gazepoint.getZ() || screenHeight * sin(screenAngle)  + GAZE_ERROR < gazepoint.getZ()
            || gazepoint.getX() > screenWidth / 2 + GAZE_ERROR || gazepoint.getX() < (-1) * screenWidth / 2 - GAZE_ERROR)
         {
            num_closest_target = num_objects;
         }
      }
      //this part should change in the next commit
      // you should use the head location to estimate whether the kid is looking at the robot
      else //num_closest_target is the index of the object named robot
      {
         if (closest_distance > 3 * GAZE_ERROR)
         {
            num_closest_target = num_objects;
         }
      }
      
      clm_ros_wrapper::DetectedTarget target;
      
      target.name = screen_reference_points_names[num_closest_target];
      target.distance = closest_distance;

      target_publisher.publish(target);
      //cout << endl << num_closest_target << endl << endl;
   }
}

void scene_callback(const clm_ros_wrapper::Scene::ConstPtr& msg)
{
   num_objects = (*msg).num_objects;
   for (int i =0; i < (*msg).num_objects ; i++)
   {
      screen_reference_points_wf[i] = tf::Vector3 ((*msg).objects[i].x_screen - screenWidth/2 + screenGap,
         cos(screenAngle) * (screenHeight - screenGap - (*msg).objects[i].y_screen),
         sin(screenAngle) * (screenHeight - screenGap - (*msg).objects[i].y_screen));

      screen_reference_points_names[i] =  std::string((*msg).objects[i].name);
   }
   screen_reference_points_names[num_objects] =  "OUTSIDE";
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "target_detector");
	ros::NodeHandle nh;

   nh.getParam("screenAngleInDegrees", screenAngleInDegrees);
   nh.getParam("screenWidth", screenWidth);
   nh.getParam("screenHeight", screenHeight);
   nh.getParam("screenGap", screenGap);

   screenAngle = screenAngleInDegrees * M_PI_2 / 90;

	target_publisher = nh.advertise<clm_ros_wrapper::DetectedTarget>("/clm_ros_wrapper/detect_target", 1);

   ros::Subscriber scene = nh.subscribe("/clm_ros_wrapper/scene", 1, &scene_callback);

	ros::Subscriber gazepoint_sub = nh.subscribe("/clm_ros_wrapper/gaze_point", 1, &gazepoint_callback);

	ros::spin();
}
