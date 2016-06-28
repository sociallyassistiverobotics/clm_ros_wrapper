
#include "CLM_core.h"
#include <std_msgs/String.h>

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
#include <limits>

#define GAZE_ERROR 50

int screenAngleInDegrees; 
float screenWidth; 
float screenHeight;
float screenAngle;

using namespace std;
using namespace cv;

using namespace boost::filesystem;

ros::Publisher region_publisher;

void gazepoint_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
   tf::Vector3 gazepoint;

	tf::vector3MsgToTF(*msg, gazepoint);

	tf::Vector3 screen_reference_points [7];

	screen_reference_points[4] = tf::Vector3((-5)* screenWidth / 12, screenHeight * cos(screenAngle) / 4, screenHeight * sin(screenAngle) / 4);
   screen_reference_points[1] = tf::Vector3((-5) * screenWidth / 12, (3) * screenHeight * cos(screenAngle) / 4, 3 * screenHeight * sin(screenAngle) / 4);
   screen_reference_points[6] = tf::Vector3(screenWidth * 5/12, (1) * screenHeight * cos(screenAngle) / 4, (1) * screenHeight * sin(screenAngle) / 4);
   screen_reference_points[3] = tf::Vector3(screenWidth * 5/12, (3) * screenHeight * cos(screenAngle) / 4, (3) * screenHeight * sin(screenAngle) / 4);
   screen_reference_points[2] = (screen_reference_points[1]+screen_reference_points[3])/2;
   screen_reference_points[5] = (screen_reference_points[4]+screen_reference_points[6])/2;

   int num_closest_region = 0;

   float closest_distance = std::numeric_limits<double>::max();

   // do a check to see if the point is inside
   // if (closest_distance > 175) // define 175 better 
   // {
   // 	num_closest_region = 0;
   // }


   for (int i = 1; i<7; i++)
   {
   	//cout<< i << "    "<< screen_reference_points[i].getX() << " " << screen_reference_points[i].getY() << "    " << screen_reference_points[i].getZ() << endl;
   	if (closest_distance > gazepoint.distance(screen_reference_points[i]))
   	{
   		closest_distance = gazepoint.distance(screen_reference_points[i]);
   		num_closest_region = i;
   	}
   }

   // do a check to see if the point is inside
   if ((-1)* GAZE_ERROR > gazepoint.getZ() || screenHeight * sin(screenAngle)  + GAZE_ERROR < gazepoint.getZ()
   	|| gazepoint.getX() > screenWidth / 2 + GAZE_ERROR || gazepoint.getX() < (-1) * screenWidth / 2 - GAZE_ERROR)
   {
   	num_closest_region = 0;
   }
   
   std_msgs::String region;
   
   std::stringstream ss;
   ss << " " << num_closest_region << " ";
   region.data = ss.str();

   region_publisher.publish(region);
   //cout << endl << num_closest_region << endl << endl;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "region_detecter");
	ros::NodeHandle nh;

   nh.getParam("region_detecter/screenAngleInDegrees", screenAngleInDegrees);
   nh.getParam("region_detecter/screenWidth", screenWidth);
   nh.getParam("region_detecter/screenHeight", screenHeight);

   screenAngle = screenAngleInDegrees * M_PI_2 / 90;

	region_publisher = nh.advertise<std_msgs::String>("/clm_ros_wrapper/detect_region", 1);

	ros::Subscriber gazepoint_sub = nh.subscribe("/clm_ros_wrapper/gaze_point", 1, &gazepoint_callback);

	ros::spin();
}
