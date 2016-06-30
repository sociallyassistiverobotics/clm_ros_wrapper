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

int screenAngleInDegrees; 
float screenWidth;
float screenHeight;
float screenAngle;

using namespace std;
using namespace cv;

using namespace boost::filesystem;

ros::Publisher gazepoint_pub;

tf::Vector3 headposition_cf;

void vector_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
   tf::Vector3 hfv_cf;

   tf::vector3MsgToTF(*msg, hfv_cf);

   //cout << screenAngle << "   " << screenWidth << "   " << screenHeight;

	// rotation matrix from camera frame to world frame
   tf::Matrix3x3 matrix_cf2wf;
   matrix_cf2wf.setValue(-1, 0, 0, 0, 0, -1, 0, -1, 0);

   tf::Matrix3x3 matrix_cf2wf_rotate_axis;

   // in the new setting, the screen is slightly rotated, so
   // I use this new matrix to do an extra rotation of the axis cf2wf
   matrix_cf2wf_rotate_axis.setValue(0.897904, 0.0145582, -0.439951,
    -0.067152, 0.992285,  -0.104216, 
    0.43504, 0.12312, 0.891954);

   //applying the extra rotation 
   matrix_cf2wf = matrix_cf2wf_rotate_axis * matrix_cf2wf;

   // translation vector from cf to wf
   tf::Vector3 vector_cf2wf = tf::Vector3((-1) * screenWidth/3, sin(screenAngle) * screenHeight, cos(screenAngle) * screenHeight);

   // transformation from the camera frame to the world frame
   tf::Transform transfrom_cf2wf = tf::Transform(matrix_cf2wf, vector_cf2wf);

   //storing the locations of the lower corners of screen and the camera 
   //in world frame to establish the space where it sits
   tf::Vector3 lower_left_corner_of_screen_wf = tf::Vector3(screenWidth / 2, 0, 0);
   tf::Vector3 lower_right_corner_of_screen_wf = tf::Vector3( -1 * screenWidth / 2, 0, 0);

   // the location of the camera in the world frame would be equal to the translation vector
   tf::Vector3 camera_wf = tf::Vector3(screenWidth/3, cos(screenAngle) * screenHeight, sin(screenAngle) * screenHeight);

   tf::Vector3 hfv_wf = matrix_cf2wf.inverse() * (hfv_cf);

   // storing the head position in the camera frame
   tf::Vector3 headposition_wf = transfrom_cf2wf(headposition_cf);

   //successful with 45 deg, 350 height and 520 width
   //cout <<  "HEAD POSITION x:" << headposition_wf.getX() << " \t y:" << headposition_wf.getY() << "\t z:" << headposition_wf.getZ() << "\n";
   
   // successful
   //cout <<  "HFV x:" << hfv_wf.getX() << " \t y:" << hfv_wf.getY() << "\t z:" << hfv_wf.getZ() << "\n";
   tf::Vector3 randompoint_on_gazedirection_wf = headposition_wf + 100 * hfv_wf;

   // using the Line-Plane intersection formula on Wolfram link: http://mathworld.wolfram.com/Line-PlaneIntersection.html
   // ALL CALCULATIONS ARE MADE IN WORLD FRAME
   // Explanation: To construct the line to intersect, I take two points in the gaze direction, the camera location and another point that is equal to camera point
   // plus a constant times the head fixation vector -- this extra point is named randompoint_on_gazedirection_wf. 
   // with the notation from the link x4 = headposition_wf and x5 = randompoint_on_gazedirection_wf
   cv::Matx<float, 4,4> matrix1 = cv::Matx<float, 4, 4>(1, 1, 1, 1,
       camera_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), headposition_wf.getX(),
       camera_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), headposition_wf.getY(),
       camera_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), headposition_wf.getZ());

   cv::Matx<float, 4,4> matrix2 = cv::Matx<float, 4, 4>(1, 1, 1, 0,
       camera_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), randompoint_on_gazedirection_wf.getX() - headposition_wf.getX(),
       camera_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), randompoint_on_gazedirection_wf.getY() - headposition_wf.getY(),
       camera_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), randompoint_on_gazedirection_wf.getZ() - headposition_wf.getZ());

   // following the formula, I calculate t -- check the link
   double determinant_ratio = (-1) * cv::determinant(matrix1) / cv::determinant(matrix2);

   // finally I plug in the determinant ratio (t) to get the intersection point
   tf::Vector3 gazepoint_on_screen_wf = headposition_wf + determinant_ratio * (randompoint_on_gazedirection_wf - headposition_wf);

   geometry_msgs::Vector3 gazepoint_msg;
   tf::vector3TFToMsg(gazepoint_on_screen_wf, gazepoint_msg);

   gazepoint_pub.publish(gazepoint_msg);
}

void headposition_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	tf::vector3MsgToTF(*msg, headposition_cf);
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "find_gazepoint");
	ros::Subscriber headposition_sub;
	ros::Subscriber vector_sub;
	ros::NodeHandle nh;

	tf::Vector3 headposition_cf;

	nh.getParam("find_gazepoint/screenAngleInDegrees", screenAngleInDegrees);
  nh.getParam("find_gazepoint/screenWidth", screenWidth);
  nh.getParam("find_gazepoint/screenHeight", screenHeight);

  screenAngle = screenAngleInDegrees * M_PI_2 / 90;

	gazepoint_pub = nh.advertise<geometry_msgs::Vector3>("clm_ros_wrapper/gaze_point", 1);

	headposition_sub = nh.subscribe("/clm_ros_wrapper/head_position", 1, &headposition_callback);
	vector_sub = nh.subscribe("/clm_ros_wrapper/head_vector", 1, &vector_callback);

	ros::spin();
}
