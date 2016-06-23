/*
This ros node subscribes to "/clm_ros_wrapper/gaze_point" and finds the region on the screen where this 
gaze point falls into. It then publishes this information with publisher topic "/clm_ros_wrapper/detect_region"
Yunus
*/

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

using namespace std;
using namespace cv;

using namespace boost::filesystem;

ros::Publisher region_publisher;

void gazepoint_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
   tf::Vector3 gazepoint;

   float screenAngle = M_PI_4; // representing 45 degrees
   float screenWidth = 520;
   float screenHeight = 320;

	tf::vector3MsgToTF(*msg, gazepoint);

   // the midpoints of the regions on the screen
   // note: screen_refence_points[0] is not defined and represents outside of the screen
	tf::Vector3 screen_reference_points [7];

	screen_reference_points[4] = tf::Vector3((-1)* screenWidth / 2, screenHeight * cos(screenAngle) / 4, screenHeight * sin(screenAngle) / 4);
   screen_reference_points[1] = tf::Vector3((-1) * screenWidth / 2, (3) * screenHeight * cos(screenAngle) / 4, 3 * screenHeight * sin(screenAngle) / 4);
   screen_reference_points[6] = tf::Vector3(screenWidth / 2, (1) * screenHeight * cos(screenAngle) / 4, (1) * screenHeight * sin(screenAngle) / 4);
   screen_reference_points[3] = tf::Vector3(screenWidth / 2, (3) * screenHeight * cos(screenAngle) / 4, (3) * screenHeight * sin(screenAngle) / 4);
   screen_reference_points[2] = (screen_reference_points[1]+screen_reference_points[3])/2;
   screen_reference_points[5] = (screen_reference_points[4]+screen_reference_points[6])/2;

   // the number value of the closest region to gazepoint
   // initialized as 0 by default
   int num_closest_region = 0;

   //the distance to this region's midpoint
   float closest_distance = std::numeric_limits<double>::max();

   for (int i = 1; i<7; i++)
   {
      // for testing
   	//cout<< i << "     "<< gazepoint.distance(screen_reference_points[i]) << endl;

      // updating the closest region
   	if (closest_distance > gazepoint.distance(screen_reference_points[i]))
   	{
   		closest_distance = gazepoint.distance(screen_reference_points[i]);
   		num_closest_region = i;
   	}
   }

   // if not less than 175, then it means, we're outside
   if (closest_distance > 175)
   {
   	num_closest_region = 0;
   }

   //using this String to publish the information
   std_msgs::String region;
   
   std::stringstream ss;
   ss << " " << num_closest_region << " ";
   region.data = ss.str();

   region_publisher.publish(region);
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "region_detecter");
	ros::NodeHandle nh;

	region_publisher = nh.advertise<std_msgs::String>("/clm_ros_wrapper/detect_region", 1);

	ros::Subscriber gazepoint_sub = nh.subscribe("/clm_ros_wrapper/gaze_point", 1, &gazepoint_callback);

	ros::spin();
}
