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
#include <clm_ros_wrapper/GazePointAndDirection.h>

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

ros::Publisher gaze_point_and_direction_pub;
ros::Publisher head_position_rf_pub;

tf::Vector3 headposition_cf;

tf::Matrix3x3 rotation_matrix_cf2wf;
tf::Vector3 translation_vector_cf2wf;

tf::Matrix3x3 rotation_matrix_wf2rf;
tf::Vector3 translation_vector_wf2rf;

std::vector<double> transformation_matrix_cf2wf_array_parameter_server;
std::vector<double> transformation_matrix_wf2rf_array_parameter_server;

void vector_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    tf::Vector3 hfv_cf;

    tf::vector3MsgToTF(*msg, hfv_cf);

    //Message to publish pd -> point and direction 
    clm_ros_wrapper::GazePointAndDirection gaze_pd_msg;

    //checking if there is a detection
    if (headposition_cf.isZero() || hfv_cf.isZero())
    {
        // no face detection publish the message with 3 zero vectors
        tf::Vector3 zero_vector = tf::Vector3(0, 0, 0);
        tf::vector3TFToMsg(zero_vector, gaze_pd_msg.gaze_point);
        tf::vector3TFToMsg(zero_vector, gaze_pd_msg.head_position);
        tf::vector3TFToMsg(zero_vector, gaze_pd_msg.hfv);
        gaze_point_and_direction_pub.publish(gaze_pd_msg);

        //for head position in the robot frame
        geometry_msgs::Vector3 zero_msg;
        tf::vector3TFToMsg(zero_vector, zero_msg);
        head_position_rf_pub.publish(zero_msg);
    }

    else 
    {

        // rotation matrix from camera frame to world frame
        
        // rotation_matrix_cf2wf.setValue(-1, 0, 0, 0, 0, -1, 0, -1, 0);

        // tf::Matrix3x3 matrix_cf2wf_rotate_axis;

        // // in the new setting, the screen is slightly rotated, so
        // // I use this new matrix to do an extra rotation of the axis cf2wf
        // rotation_matrix_cf2wf_rotate_axis.setValue(0.897904, 0.0145582, -0.439951,
        //     -0.067152, 0.992285,  -0.104216, 
        //     0.43504, 0.12312, 0.891954);

        //applying the extra rotation 
        // matrix_cf2wf = matrix_cf2wf_rotate_axis * matrix_cf2wf;

        // translation vector from cf to wf
        //tf::Vector3 vector_cf2wf; //tf::Vector3((-1) * screenWidth/3, sin(screenAngle) * screenHeight, cos(screenAngle) * screenHeight);

        // transformation from the camera frame to the world frame
        tf::Transform transfrom_cf2wf = tf::Transform(rotation_matrix_cf2wf, translation_vector_cf2wf);

        //storing the locations of the lower corners of screen and the camera 
        //in world frame to establish the space where it sits
        tf::Vector3 lower_left_corner_of_screen_wf = tf::Vector3(screenWidth / 2, 0, 0);
        tf::Vector3 lower_right_corner_of_screen_wf = tf::Vector3( -1 * screenWidth / 2, 0, 0);

        // the location of the camera in the world frame would be equal to the translation vector
        tf::Vector3 camera_wf = tf::Vector3(screenWidth/3, cos(screenAngle) * screenHeight, sin(screenAngle) * screenHeight);

        tf::Vector3 hfv_wf = rotation_matrix_cf2wf.inverse() * (hfv_cf);

        // testing
        //headposition_cf = tf::Vector3(-82, 350, 260);

        // storing the head position in the camera frame
        tf::Vector3 headposition_wf = transfrom_cf2wf(headposition_cf);

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

        tf::vector3TFToMsg(gazepoint_on_screen_wf, gaze_pd_msg.gaze_point);
        tf::vector3TFToMsg(headposition_wf, gaze_pd_msg.head_position);
        tf::vector3TFToMsg(hfv_wf, gaze_pd_msg.hfv);
        gaze_point_and_direction_pub.publish(gaze_pd_msg);

        //tranforming the head position to robot frame
        tf::Transform transformation_wf2rf = tf::Transform(rotation_matrix_wf2rf, translation_vector_wf2rf);
        tf::Vector3 head_position_rf = transformation_wf2rf(headposition_wf);

        geometry_msgs::Vector3 head_position_rf_msg;
        tf::vector3TFToMsg(head_position_rf, head_position_rf_msg);

        head_position_rf_pub.publish(head_position_rf_msg);

        tf::Vector3 zero_vector = tf::Vector3(0,0,0);
        headposition_cf = zero_vector;
        hfv_cf = zero_vector;
    }
}

void headposition_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    tf::vector3MsgToTF(*msg, headposition_cf);
    headposition_cf = headposition_cf + tf::Vector3(0,0,60);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "find_gazepoint");
    ros::Subscriber headposition_sub;
    ros::Subscriber vector_sub;
    ros::NodeHandle nh;

    // Screen parameters
    nh.getParam("screenAngleInDegrees", screenAngleInDegrees);
    nh.getParam("screenWidth", screenWidth);
    nh.getParam("screenHeight", screenHeight);

    // loading rotation matrix from cf to wf from the parameter server
    // this rotation matrix depends on the rotation of the screen

    // here I load the values in headRotationMatrixCLM_cf of type Matx33d to an array and 
    // load the values in the array to a matrix of type tf::Matrix3x3

    // setFromOPenGLSubMatrix (used below) is defined as follows and skips one element i.e. m[3] (the code might have an error)
    // so I use a 12 element array instead of 9, and increment the index by 4 each iteration instead of 3

    // from the source code of setFromOpenGLSubMatrix
    // void setFromOpenGLSubMatrix(const tfScalar *m)
    // {
    //     m_el[0].setValue(m[0],m[4],m[8]);
    //     m_el[1].setValue(m[1],m[5],m[9]);
    //     m_el[2].setValue(m[2],m[6],m[10]);
    // }

    // loading transformation matrix from cf to wf from the parameter server
    nh.getParam("transformation_cf2wf", transformation_matrix_cf2wf_array_parameter_server);
    
    tfScalar tf_scalar_row_major_transformation_matrix_array_cf2wf [16];

    for(int i = 0; i < 4 ; i++)
    {
        for(int j = 0; j<4; j++)
        {
            tf_scalar_row_major_transformation_matrix_array_cf2wf[4*j+i] = (tfScalar) transformation_matrix_cf2wf_array_parameter_server[4*i+j];
        }
    }

    rotation_matrix_cf2wf.setFromOpenGLSubMatrix(tf_scalar_row_major_transformation_matrix_array_cf2wf);

    // for (int i = 0; i<3; i++)
    // {
    //     cout << endl << rotation_matrix_cf2wf.getRow(i).getX() << "   " << rotation_matrix_cf2wf.getRow(i).getY() << "   " << rotation_matrix_cf2wf.getRow(i).getZ() << endl; 
    // }

    //constructing the translation vector object using the values from the array
    translation_vector_cf2wf.setX(transformation_matrix_cf2wf_array_parameter_server[12]);
    translation_vector_cf2wf.setY(transformation_matrix_cf2wf_array_parameter_server[13]);
    translation_vector_cf2wf.setZ(transformation_matrix_cf2wf_array_parameter_server[14]); //CHECK THESE INDICES!!!

    nh.getParam("transformation_wf2rf", transformation_matrix_wf2rf_array_parameter_server);
    
    tfScalar tf_scalar_row_major_transformation_matrix_array_wf2rf [16];

    for(int i = 0; i < 4 ; i++)
    {
        for(int j = 0; j<4; j++)
        {
            tf_scalar_row_major_transformation_matrix_array_wf2rf[4*j+i] = (tfScalar) transformation_matrix_wf2rf_array_parameter_server[4*i+j];
        }
    }

    rotation_matrix_cf2wf.setFromOpenGLSubMatrix(tf_scalar_row_major_transformation_matrix_array_wf2rf);

    //constructing the translation vector object using the values from the array
    translation_vector_cf2wf.setX(transformation_matrix_wf2rf_array_parameter_server[12]);
    translation_vector_cf2wf.setY(transformation_matrix_wf2rf_array_parameter_server[13]);
    translation_vector_cf2wf.setZ(transformation_matrix_wf2rf_array_parameter_server[14]);

    screenAngle = screenAngleInDegrees * M_PI_2 / 90;

    gaze_point_and_direction_pub = nh.advertise<clm_ros_wrapper::GazePointAndDirection>("clm_ros_wrapper/gaze_point_and_direction", 1);

    head_position_rf_pub = nh.advertise<geometry_msgs::Vector3>("clm_ros_wrapper/head_position_rf",1);

    headposition_sub = nh.subscribe("/clm_ros_wrapper/head_position", 1, &headposition_callback);
    vector_sub = nh.subscribe("/clm_ros_wrapper/head_vector", 1, &vector_callback);

    ros::spin();
}
