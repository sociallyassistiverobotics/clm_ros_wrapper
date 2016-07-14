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
#include <clm_ros_wrapper/VectorWithCertainty.h>

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
using std::vector;

ros::Publisher gaze_point_and_direction_pub;
ros::Publisher head_position_rf_pub;

tf::Vector3 headposition_cf;

cv::Matx<float, 4, 4> transformation_cf2intermediate_frame;
cv::Matx<float, 4, 4> transformation_intermediate_frame2wf;
cv::Matx<float, 4, 4> transformation_wf2rf;

std::vector<double> transformation_matrix_cf2intermediate_frame_array_parameter_server;
std::vector<double> transformation_matrix_intermediate_frame2wf_array_parameter_server;
std::vector<double> transformation_matrix_wf2rf_array_parameter_server;

float detection_certainty;

tf::Vector3 vector3_cv2tf(cv::Matx<float, 4, 1> vector_cv)
{
    tf::Vector3 vector_tf;
    vector_tf.setX(vector_cv(0,0));
    vector_tf.setY(vector_cv(1,0));
    vector_tf.setZ(vector_cv(2,0));
    return vector_tf;
}

cv::Matx<float, 4, 1> vector3_tf2cv(tf::Vector3 vector_tf, bool isPoint)
{
    if (isPoint)
    {
        return cv::Matx<float, 4, 1>(vector_tf.getX(), vector_tf.getY(), vector_tf.getZ(), 1);
    }
    else
    {
        return cv::Matx<float, 4, 1>(vector_tf.getX(), vector_tf.getY(), vector_tf.getZ(), 0);
    }
}

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
        gaze_pd_msg.certainty = detection_certainty;
        gaze_point_and_direction_pub.publish(gaze_pd_msg);

        //for head position in the robot frame
        geometry_msgs::Vector3 zero_msg;
        tf::vector3TFToMsg(zero_vector, zero_msg);

        clm_ros_wrapper::VectorWithCertainty head_position_rf_with_certainty;
        head_position_rf_with_certainty.position = zero_msg;
        head_position_rf_with_certainty.certainty = detection_certainty;
        head_position_rf_pub.publish(head_position_rf_with_certainty);
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
        // tf::Transform transfrom_cf2wf = tf::Transform(rotation_matrix_cf2wf, translation_vector_cf2wf);

        cv::Matx<float,4,4> transformation_matrix_cf2wf = transformation_cf2intermediate_frame * transformation_intermediate_frame2wf;

        tf::Vector3 hfv_wf = vector3_cv2tf(transformation_matrix_cf2wf * (vector3_tf2cv(hfv_cf, 0)));
        //cout << vector3_tf2cv(hfv_cf, 0) << endl;


        // correcting the Z element of the head fixation vector from CLM
        hfv_wf.setZ(hfv_wf.getZ() -0.2);

        // testing
        //headposition_cf = tf::Vector3(-82, 350, 260);

        // storing the head position in the camera frame
        // /headposition_cf = tf::Vector3(0,0,500);

        // adding the box size = 200mm
        headposition_cf = headposition_cf + tf::Vector3(0,0,200);

        cv::Matx<float, 4, 1> headposition_wf_cv = transformation_matrix_cf2wf.inv() * (vector3_tf2cv(headposition_cf, 1));
        tf::Vector3 headposition_wf = vector3_cv2tf(headposition_wf_cv);

        //head position robot frame
        tf::Vector3 head_position_rf = vector3_cv2tf(transformation_wf2rf.inv()*vector3_tf2cv(headposition_wf, 1));

        //publishing the head position in the robot frame
        clm_ros_wrapper::VectorWithCertainty head_position_rf_with_certainty;

        geometry_msgs::Vector3 head_position_rf_msg;
        tf::vector3TFToMsg(head_position_rf, head_position_rf_msg); 

        head_position_rf_with_certainty.position = head_position_rf_msg;
        head_position_rf_with_certainty.certainty = detection_certainty;

        head_position_rf_pub.publish(head_position_rf_with_certainty);

        tf::Vector3 randompoint_on_gazedirection_wf = headposition_wf + 100 * hfv_wf;

        //storing the locations of the lower corners of screen and the camera 
        //in world frame to establish the space where it sits
        tf::Vector3 lower_left_corner_of_screen_wf = tf::Vector3(screenWidth / 2, 0, 0);
        tf::Vector3 lower_right_corner_of_screen_wf = tf::Vector3( -1 * screenWidth / 2, 0, 0);
        tf::Vector3 upper_mid__point_of_screen_wf = tf::Vector3(0,cos(screenAngle) * screenHeight, sin(screenAngle) * screenHeight);

        // using the Line-Plane intersection formula on Wolfram link: http://mathworld.wolfram.com/Line-PlaneIntersection.html
        // ALL CALCULATIONS ARE MADE IN WORLD FRAME
        // Explanation: To construct the line to intersect, I take two points in the gaze direction, the camera location and another point that is equal to camera point
        // plus a constant times the head fixation vector -- this extra point is named randompoint_on_gazedirection_wf. 
        // with the notation from the link x4 = headposition_wf and x5 = randompoint_on_gazedirection_wf
        cv::Matx<float, 4,4> matrix1 = cv::Matx<float, 4, 4>(1, 1, 1, 1,
            upper_mid__point_of_screen_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), headposition_wf.getX(),
            upper_mid__point_of_screen_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), headposition_wf.getY(),
            upper_mid__point_of_screen_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), headposition_wf.getZ());

        cv::Matx<float, 4,4> matrix2 = cv::Matx<float, 4, 4>(1, 1, 1, 0,
            upper_mid__point_of_screen_wf.getX(), lower_right_corner_of_screen_wf.getX(), lower_left_corner_of_screen_wf.getX(), randompoint_on_gazedirection_wf.getX() - headposition_wf.getX(),
            upper_mid__point_of_screen_wf.getY(), lower_right_corner_of_screen_wf.getY(), lower_left_corner_of_screen_wf.getY(), randompoint_on_gazedirection_wf.getY() - headposition_wf.getY(),
            upper_mid__point_of_screen_wf.getZ(), lower_right_corner_of_screen_wf.getZ(), lower_left_corner_of_screen_wf.getZ(), randompoint_on_gazedirection_wf.getZ() - headposition_wf.getZ());

        // following the formula, I calculate t -- check the link
        double determinant_ratio = (-1) * cv::determinant(matrix1) / cv::determinant(matrix2);

        // finally I plug in the determinant ratio (t) to get the intersection point
        tf::Vector3 gazepoint_on_screen_wf = headposition_wf + determinant_ratio * (randompoint_on_gazedirection_wf - headposition_wf);

        tf::vector3TFToMsg(gazepoint_on_screen_wf, gaze_pd_msg.gaze_point);
        tf::vector3TFToMsg(headposition_wf, gaze_pd_msg.head_position);
        tf::vector3TFToMsg(hfv_wf, gaze_pd_msg.hfv);
        gaze_pd_msg.certainty = detection_certainty;
        gaze_point_and_direction_pub.publish(gaze_pd_msg);

        tf::Vector3 zero_vector = tf::Vector3(0,0,0);
        headposition_cf = zero_vector;
        hfv_cf = zero_vector;
    }
    cout << "detection_certainty" << detection_certainty << endl;
}

void headposition_callback(const clm_ros_wrapper::VectorWithCertainty::ConstPtr& msg)
{
    tf::vector3MsgToTF((*msg).position, headposition_cf);
    detection_certainty = (*msg).certainty;
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
    nh.getParam("transformation_cf2intermediate_frame", transformation_matrix_cf2intermediate_frame_array_parameter_server);
    
    for (int i = 0; i <4; i++)
    {
        for(int j =0; j <4; j++)
        {
            transformation_cf2intermediate_frame(i,j) = transformation_matrix_cf2intermediate_frame_array_parameter_server[4*i+j];
        }
    }

    nh.getParam("transformation_intermediate_frame2wf", transformation_matrix_intermediate_frame2wf_array_parameter_server);

    for (int i = 0; i <4; i++)
    {
        for(int j =0; j <4; j++)
        {
            transformation_intermediate_frame2wf(i,j) = transformation_matrix_intermediate_frame2wf_array_parameter_server[4*i+j];
        }
    }

    nh.getParam("transformation_wf2rf", transformation_matrix_wf2rf_array_parameter_server);

    for (int i = 0; i <4; i++)
    {
        for(int j =0; j <4; j++)
        {
            transformation_wf2rf(i,j) = transformation_matrix_wf2rf_array_parameter_server[4*i+j];
        }
    }

    screenAngle = screenAngleInDegrees * M_PI_2 / 90;

    gaze_point_and_direction_pub = nh.advertise<clm_ros_wrapper::GazePointAndDirection>("clm_ros_wrapper/gaze_point_and_direction", 1);

    head_position_rf_pub = nh.advertise<clm_ros_wrapper::VectorWithCertainty>("clm_ros_wrapper/head_position_rf",1);

    headposition_sub = nh.subscribe("/clm_ros_wrapper/head_position", 1, &headposition_callback);
    vector_sub = nh.subscribe("/clm_ros_wrapper/head_vector", 1, &vector_callback);

    ros::spin();
}
