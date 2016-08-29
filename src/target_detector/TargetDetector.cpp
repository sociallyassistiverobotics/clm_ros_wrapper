
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
#include <clm_ros_wrapper/GazePointAndDirection.h>

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h>
#include <limits>

#define max_num_objects 20

int screenAngleInDegrees;
float screenWidth;
float screenHeight;
float screenAngle;
float screenGap;
string _namespace;


int num_objects_on_screen, num_free_objects;

//the positions are loaded to these arrays
tf::Vector3 screen_reference_points_wf [max_num_objects];
tf::Vector3 free_objects_positions [max_num_objects];

//the objects' names are here with the same index as they have
//in the array for positions
std::string screen_reference_points_names [max_num_objects];
std::string free_objects_names [max_num_objects];

//to do in/out checks for the free objects
float free_object_radius;

float robot_position_wf_x, robot_position_wf_y, robot_position_wf_z;

ros::Publisher target_publisher;

using namespace std;

void gazepoint_callback(const clm_ros_wrapper::GazePointAndDirection::ConstPtr& msg)
{
    double detection_certainty = (*msg).certainty;
    tf::Vector3 gaze_point_wf, head_position_wf, hfv_wf;

    //checking if there is detection
    tf::vector3MsgToTF((*msg).gaze_point, gaze_point_wf);
    tf::vector3MsgToTF((*msg).head_position, head_position_wf);
    tf::vector3MsgToTF((*msg).hfv, hfv_wf);

    if (gaze_point_wf.isZero() && head_position_wf.isZero() && hfv_wf.isZero())
    {
        //means no detection
        clm_ros_wrapper::DetectedTarget target_no_detection;
        target_no_detection.certainty = detection_certainty;

        target_no_detection.name = "NO DETECTION";
        target_no_detection.distance = 0;
        target_no_detection.region = target_no_detection.NONE;

        target_publisher.publish(target_no_detection);
    }

    else
    {
        clm_ros_wrapper::DetectedTarget detected_target;
        detected_target.certainty = detection_certainty;

        int num_closest_object_on_screen = 0, num_closest_free_object = 0;

        // TODO: cmhuang: change this..
        // to make sure this callback happens after scene_callback
        if (num_objects_on_screen != 0 || num_free_objects != 0)
        {

            float closest_distance_screen = std::numeric_limits<double>::max();
            float closest_distance_free_object = std::numeric_limits<double>::max();

            // find the closet object displayed on the screen
            for (int i = 0; i<num_objects_on_screen; i++)
            {
                if (closest_distance_screen > gaze_point_wf.distance(screen_reference_points_wf[i]))
                {
                    closest_distance_screen = gaze_point_wf.distance(screen_reference_points_wf[i]);
                    num_closest_object_on_screen = i;
                }
            }

            //checking for the  boundaries of the screen in Z and X axes to see
            //if the gazepoint is outside of the screen
            //3 *screenGap on the bottom because the gap is bigger
            if ((3) * screenGap * sin(screenAngle) > gaze_point_wf.getZ() || (screenHeight  - screenGap)*sin(screenAngle) < gaze_point_wf.getZ()
                || gaze_point_wf.getX() > screenWidth / 2 - screenGap || gaze_point_wf.getX() < (-1) * screenWidth / 2 +  screenGap)
            {
                num_closest_object_on_screen = num_objects_on_screen;
                // the index num_closest_object_on_screen refers to outside -- is the object named "Outside"
                closest_distance_screen = std::numeric_limits<double>::max();
            }

            //USING THE LINE-POINT DISTANCE FORMULA TO FIND THE CLOSEST FREE OBJECT
            // To see the calculations in more depth: http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
            // Explanation:
            // X_1: head_position_wf, X_2:randompoint_on_gazedirection X_0:free object's position

            tf::Vector3 randompoint_on_gazedirection = head_position_wf + 1000 * hfv_wf;

            //dummy zero vector because the function length is defined for quaternions only ???
            tf::Vector3 zero_vector = tf::Vector3(0,0,0);

            for (int i = 0; i < num_free_objects; i++)
            {
                tf::Vector3 diff_freeobj_headpos = free_objects_positions[i]-head_position_wf;
                tf::Vector3 diff_freeobj_rand = free_objects_positions[i]-randompoint_on_gazedirection;

                //using the formula from the link
                // float distance = zero_vector.distance(diff_freeobj_headpos.cross(diff_freeobj_rand))\
                //     /zero_vector.distance(randompoint_on_gazedirection - head_position_wf);
                float distance = zero_vector.distance(diff_freeobj_headpos.cross(diff_freeobj_rand))\
                    /zero_vector.distance(randompoint_on_gazedirection - head_position_wf);

                if (closest_distance_free_object > distance)
                {
                    closest_distance_free_object = distance;
                    num_closest_free_object = i;
                }
            }
            // inside/outside check for the closest free object
            // free_object_radius is the robot radius
            // TODO: change this parameter
            if (closest_distance_free_object > free_object_radius)
            {
                num_closest_free_object = num_free_objects;
                closest_distance_free_object = std::numeric_limits<double>::max();
                //setting it to "OUTSIDE"
            }

            // This part changes because of the new free object message type
            // if (screen_reference_points_names[num_closest_target].compare("robot")!=0)
            // {
            // // WHAT IF THE POINT IS OUTSIDE THE SCREEN
            // // do a check to see if the point is inside
            //     if ((-1)* screenGap > gazepoint.getZ() || screenHeight * sin(screenAngle)  + screenGap < gazepoint.getZ()
            //         || gazepoint.getX() > screenWidth / 2 + screenGap || gazepoint.getX() < (-1) * screenWidth / 2 - screenGap)
            //     {
            //         num_closest_target = num_objects;
            //     }
            // }
            // //this part should change in the next commits
            // // you should use the head location to estimate whether the kid is looking at the robot
            // else //num_closest_target is the index of the object named robot
            // {
            //     if (closest_distance > 3 * screenGap)
            //     {
            //         num_closest_target = num_objects;
            //     }
            // }


            if (screen_reference_points_names[num_closest_object_on_screen].compare("OUTSIDE") == 0
                && free_objects_names[num_closest_free_object].compare("OUTSIDE") == 0)
            {
                detected_target.name = "OUTSIDE";
                detected_target.distance = 0.0;
                detected_target.region = detected_target.OUTSIDE;
            }
            else
            {
                // checking between free objects and screen objects to find the closest
                if (closest_distance_free_object > closest_distance_screen)
                {
                    detected_target.name = screen_reference_points_names[num_closest_object_on_screen];
                    detected_target.distance = closest_distance_screen;
                    detected_target.region = detected_target.SCREEN;
                }
                else
                {
                    detected_target.name = free_objects_names[num_closest_free_object];
                    detected_target.distance = closest_distance_free_object;
                    if(detected_target.name == "robot"){
                        detected_target.region = detected_target.ROBOT;
                    }
                    else if(detected_target.name == "parent"){
                        detected_target.region = detected_target.PARENT;
                    }
                }
            }

            //target_publisher.publish(detected_target);
            //cout << endl << num_closest_target << endl << endl;
        }

        //clm_ros_wrapper::DetectedTarget detected_target;

        // if (num_closest_object_on_screen == num_objects_on_screen) // means the point is outside the screen
        // {
        //     detected_target.region = detected_target.OUTSIDE;
        // }
        // else
        // {
        //     detected_target.region = detected_target.SCREEN;
        // }

        // TODO: the robot position should come from the scene publisher?
        // cmhuang: I do not know why we need the following code
        // tf::Vector3 robot_position_tf = tf::Vector3(robot_position_wf_x, robot_position_wf_y, robot_position_wf_z);

        // tf::Vector3 randompoint_on_gazedirection = head_position_wf + 1000 * hfv_wf;

        // tf::Vector3 diff_robotpos_headpos = robot_position_tf-head_position_wf;
        // tf::Vector3 diff_robotpos_rand = robot_position_tf-randompoint_on_gazedirection;

        // //using the formula from the link
        // float distance = tf::Vector3(0,0,0).distance(diff_robotpos_headpos.cross(diff_robotpos_rand))/tf::Vector3(0,0,0).distance(randompoint_on_gazedirection - head_position_wf);
        // if (distance < free_object_radius)
        // {
        //     detected_target.region = detected_target.ROBOT;
        // }

        // else if (detected_target.region == detected_target.SCREEN)
        // {
        //     detected_target.region = detected_target.SCREEN;
        // }

        // else
        // {
        //     detected_target.region = detected_target.OUTSIDE;
        // }

        target_publisher.publish(detected_target);
    }
}

void gazepoint_callback2(const clm_ros_wrapper::GazePointAndDirection::ConstPtr& msg)
{
    double detection_certainty = (*msg).certainty;
    tf::Vector3 gaze_point_wf, head_position_wf, hfv_wf;

    //checking if there is detection
    tf::vector3MsgToTF((*msg).gaze_point, gaze_point_wf);
    tf::vector3MsgToTF((*msg).head_position, head_position_wf);
    tf::vector3MsgToTF((*msg).hfv, hfv_wf);

    if (gaze_point_wf.isZero() && head_position_wf.isZero() && hfv_wf.isZero())
    {
        //means no detection
        clm_ros_wrapper::DetectedTarget target_no_detection;
        target_no_detection.certainty = detection_certainty;

        target_no_detection.name = "NO DETECTION";
        target_no_detection.distance = 0;
        target_no_detection.region = target_no_detection.NONE;

        target_publisher.publish(target_no_detection);
    }

    else
    {
        clm_ros_wrapper::DetectedTarget detected_target;
        detected_target.certainty = detection_certainty;

        tf::Vector3 virtual_screen = tf::Vector3(270,250,240);
        tf::Vector3 virtual_robot = tf::Vector3(640,170,300);
        tf::Vector3 virtual_parent = tf::Vector3(750,-300,400);
        float _height = 500;
        float _radian = 0.523599; //30 degree
        if(is_point_inside_cone(head_position_wf, hfv_wf, _height, _radian, virtual_parent))
            std::cout << "parent" << std::endl;
        else if(is_point_inside_cone(head_position_wf, hfv_wf, _height, _radian, virtual_robot))
            std::cout << "robot" << std::endl;
        else if(is_point_inside_cone(head_position_wf, hfv_wf, _height, _radian, virtual_screen))
            std::cout << "screen" << std::endl;
        else
            std::cout << "others" << std::endl;




        //target_publisher.publish(detected_target);
    }
}

bool is_point_inside_cone(tf::Vector3 top_point, tf::Vector3 direction_vec, float height, float radian, tf::Vector3 test_point)
{
    // check 1: distance to line is greater than radius of the cone
    // http://math.harvard.edu/~ytzeng/worksheet/distance.pdf
    tf::Vector3 p = test_point;
    tf::Vector3 q = top_point;
    tf::Vector3 pq = q - p;
    float cross_value = (pq.cross(direction_vec)).length();
    float unit_vec_length = direction_vec.length()
    float d = cross_value / unit_vec_length;
    float r = height * tan(radian);
    if(d > r) return false;

    // check 2: the projected point is inside the directional line
    // http://stackoverflow.com/questions/17581738/check-if-a-point-projected-on-a-line-segment-is-not-outside-it
    bottom_point = top_point + height * direction_vec;
    tf::Vector3 attention_directional_line = bottom_point - top_point;
    tf::Vector3 pq = p - q;
    double innerProduct = pq.dot(attention_directional_line);
    bool inside_line = (0 <= innerProduct && innerProduct <= attention_directional_line.dot(attention_directional_line));
    if(inside_line == false) return false;

    // check 3: distance to line is within the cone triangle
    double d_prime = innerProduct / attention_directional_line.length();
    double r_prime = d_prime * tan(radian);
    if(d > r_prime) return false;
    return true;
}

void scene_callback(const clm_ros_wrapper::Scene::ConstPtr& msg)
{
    num_objects_on_screen = (*msg).screen.num_objects_on_screen;
    num_free_objects = (*msg).num_free_objects;

    // the objects on the screen
    for (int i =0; i < (*msg).screen.num_objects_on_screen ; i++)
    {
        //converting 2D coordinates of points on the screen to 3D points in the world frame
        screen_reference_points_wf[i] = tf::Vector3 ((*msg).screen.objects_on_screen[i].x_screen - screenWidth/2 + screenGap,
            cos(screenAngle) * (screenHeight - screenGap - (*msg).screen.objects_on_screen[i].y_screen),
            sin(screenAngle) * (screenHeight - screenGap - (*msg).screen.objects_on_screen[i].y_screen));

       screen_reference_points_names[i] =  std::string((*msg).screen.objects_on_screen[i].name);
    }

    screen_reference_points_names[(*msg).screen.num_objects_on_screen] =  "OUTSIDE";

    // the free objects
    for (int i =0; i < (*msg).num_free_objects; i++)
    {
        //free objects' positions are already given in world frame
        tf::vector3MsgToTF((*msg).free_objects[i].position, free_objects_positions[i]);
        free_objects_names[i] = std::string((*msg).free_objects[i].name);
    }

    free_objects_names[(*msg).num_free_objects] =  "OUTSIDE";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_detector");

    ros::NodeHandle nh;

    nh.getParam("ns", _namespace);

    nh.getParam("screenAngleInDegrees", screenAngleInDegrees);
    nh.getParam("screenWidth", screenWidth);
    nh.getParam("screenHeight", screenHeight);
    nh.getParam("screenGap", screenGap);

    std::vector<float> transformation_wf2rf_param_ser;

    nh.getParam("transformation_wf2rf", transformation_wf2rf_param_ser);

    float robot_radius = robot_position_wf_z;
    free_object_radius = robot_radius; // TODO: need to change this parameter

    nh.getParam("robot_x_wf", robot_position_wf_x);
    nh.getParam("robot_y_wf", robot_position_wf_y);
    nh.getParam("robot_z_wf", robot_position_wf_z);

    screenAngle = screenAngleInDegrees * M_PI_2 / 90;

    target_publisher = nh.advertise<clm_ros_wrapper::DetectedTarget>("/sar/perception/detect_target", 1);

    ros::Subscriber scene = nh.subscribe("/sar/perception/scene", 1, &scene_callback);

    ros::Subscriber gazepoint_sub = nh.subscribe("/sar/perception/gaze_point_and_direction_wf", 1, &gazepoint_callback2);

    ros::spin();
}
