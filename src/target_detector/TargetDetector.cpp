
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

#define GAZE_ERROR 50
#define max_num_objects 20

int screenAngleInDegrees; 
float screenWidth; 
float screenHeight;
float screenAngle;
float screenGap;


int num_objects_on_screen, num_free_objects;

//the positions are loaded to these arrays
tf::Vector3 screen_reference_points_wf [max_num_objects];
tf::Vector3 free_objects_positions [max_num_objects];

//the objects' names are here with the same index as they have
//in the array for positions
std::string screen_reference_points_names [max_num_objects];
std::string free_objects_names [max_num_objects];

//to do in/out checks for the free objects
float free_object_radii [max_num_objects];

ros::Publisher target_publisher;

using namespace std;

void gazepoint_callback(const clm_ros_wrapper::GazePointAndDirection::ConstPtr& msg)
{
    // to make sure this callback happens after scene_callback  
    if (num_objects_on_screen != 0 || num_free_objects != 0)
    {
        tf::Vector3 gaze_point_wf, head_position_wf, hfv_wf;

        //checking if there is detection
        tf::vector3MsgToTF((*msg).gaze_point, gaze_point_wf);
        tf::vector3MsgToTF((*msg).head_position, head_position_wf);
        tf::vector3MsgToTF((*msg).hfv, hfv_wf);

        if (gaze_point_wf.isZero() && head_position_wf.isZero() && hfv_wf.isZero())
        {   
            //means no detection
            clm_ros_wrapper::DetectedTarget target_no_detection;

            target_no_detection.name = "NO DETECTION";
            target_no_detection.distance = 0;

            target_publisher.publish(target_no_detection);
        }

        else
        {
            int num_closest_object_on_screen = 0, num_closest_free_object = 0;

            float closest_distance_screen = std::numeric_limits<double>::max();
            float closest_distance_free_object = std::numeric_limits<double>::max();


            //inside/outside check for the screen
            if ((-1)* GAZE_ERROR > gaze_point_wf.getZ() || screenHeight * sin(screenAngle)  + GAZE_ERROR < gaze_point_wf.getZ()
                || gaze_point_wf.getX() > screenWidth / 2 + GAZE_ERROR || gaze_point_wf.getX() < (-1) * screenWidth / 2 - GAZE_ERROR)
            {
                num_closest_object_on_screen = num_objects_on_screen;
            }
            else
            {
                for (int i = 0; i<num_objects_on_screen; i++)
                {
                    if (closest_distance_screen > gaze_point_wf.distance(screen_reference_points_wf[i]))
                    {
                        closest_distance_screen = gaze_point_wf.distance(screen_reference_points_wf[i]);
                        num_closest_object_on_screen = i;
                    }
                }
            }

            //USING THE LINE-POINT DISTANCE FORMULA TO FIND THE CLOSEST FREE OBJECT
            // To see the calculations in more depth: http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
            // Explanation:
            // X_1: head_position_wf, X_2:randompoint_on_gazedirection X_0:free object's position

            tf::Vector3 randompoint_on_gazedirection = head_position_wf + 100 * hfv_wf;

            //dummy zero vector because the function length is defined for quaternions only ???
            tf::Vector3 zero_vector = tf::Vector3(0,0,0);

            for (int i = 0; i < num_free_objects; i++)
            {   
                tf::Vector3 diff_freeobj_headpos = free_objects_positions[i]-head_position_wf;
                tf::Vector3 diff_freeobj_rand = free_objects_positions[i]-randompoint_on_gazedirection;

                //using the formula from the link
                float distance = zero_vector.distance(diff_freeobj_headpos.cross(diff_freeobj_rand))\
                    /zero_vector.distance(randompoint_on_gazedirection - head_position_wf);

                if (closest_distance_free_object > distance)
                {
                    closest_distance_free_object = distance;
                    num_closest_free_object = i;
                }
            }

            // inside/outside check for the closest free object
            float closest_free_object_radius;
            
            if (closest_distance_free_object > free_object_radii[num_closest_free_object])
            {
                num_closest_free_object = num_free_objects;
                //setting it to "OUTSIDE"
            }

            // This part changes because of the new free object message type
            // if (screen_reference_points_names[num_closest_target].compare("robot")!=0)
            // {
            // // WHAT IF THE POINT IS OUTSIDE THE SCREEN
            // // do a check to see if the point is inside
            //     if ((-1)* GAZE_ERROR > gazepoint.getZ() || screenHeight * sin(screenAngle)  + GAZE_ERROR < gazepoint.getZ()
            //         || gazepoint.getX() > screenWidth / 2 + GAZE_ERROR || gazepoint.getX() < (-1) * screenWidth / 2 - GAZE_ERROR)
            //     {
            //         num_closest_target = num_objects;
            //     }
            // }
            // //this part should change in the next commits
            // // you should use the head location to estimate whether the kid is looking at the robot
            // else //num_closest_target is the index of the object named robot
            // {
            //     if (closest_distance > 3 * GAZE_ERROR)
            //     {
            //         num_closest_target = num_objects;
            //     }
            // }

            clm_ros_wrapper::DetectedTarget detected_target;

            // checking between free objects and screen objects to find the closest
            if (closest_distance_free_object > closest_distance_screen)
            {
                detected_target.name = screen_reference_points_names[num_closest_object_on_screen];
                detected_target.distance = closest_distance_screen;
            }
            else
            {
                detected_target.name = free_objects_names[num_closest_free_object];
                detected_target.distance = closest_distance_free_object;
            }

            target_publisher.publish(detected_target);
            //cout << endl << num_closest_target << endl << endl;
        }
    }
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

    nh.getParam("screenAngleInDegrees", screenAngleInDegrees);
    nh.getParam("screenWidth", screenWidth);
    nh.getParam("screenHeight", screenHeight);
    nh.getParam("screenGap", screenGap);

    screenAngle = screenAngleInDegrees * M_PI_2 / 90;

    target_publisher = nh.advertise<clm_ros_wrapper::DetectedTarget>("/clm_ros_wrapper/detect_target", 1);

    ros::Subscriber scene = nh.subscribe("/clm_ros_wrapper/scene", 1, &scene_callback);

    if (free_objects_names[0] != NULL)
    {
        for (int i = 0; i < num_free_objects; i++)
        {
            nh.getParam(free_objects_names[i] + "_radius", free_object_radii[i]);
        }
    }

    ros::Subscriber gazepoint_sub = nh.subscribe("/clm_ros_wrapper/gaze_point_and_direction", 1, &gazepoint_callback);

    ros::spin();
}
