///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, University of Cambridge,
// all rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "CLM_core.h"
#include <std_msgs/String.h>


#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/face.hpp>
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
#include <clm_ros_wrapper/GazeDirection.h>
#include <clm_ros_wrapper/ClmFacialActionUnit.h>
#include <clm_ros_wrapper/VectorWithCertainty.h>

#include <clm_ros_wrapper/ClmEyeGazes.h>
#include <clm_ros_wrapper/GazeDirections.h>
#include <clm_ros_wrapper/VectorsWithCertainty.h>
#include <clm_ros_wrapper/ClmHeadVectors.h>

#include <clm_ros_wrapper/Assessment.h>

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>

#include <tf/transform_datatypes.h>
#include <sar_core/SystemState.h>

// call back on the mouse click event on the image
static void mouse_callback(int event, int x, int y, int flags, void* userdata);

class ClmWrapper
{
private:
    std::string name;
    std::string executable_location;

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;

    // subscriber for the parent role
    ros::Subscriber parentRoleSubscriber;

    ros::Subscriber system_sub;

    // publisher for the detected face images
    image_transport::Publisher imagePublisher;

    // can be called through the "clm_ros_wrapper/heads" topic
    ros::Publisher headsPublisher;

    // publishing the head fixation vector
    ros::Publisher hfv_publisher;

     // publishing head position in the camera frame
    ros::Publisher head_position_publisher;

    // publishing eye gaze
    ros::Publisher eye_gaze_publisher;

    ros::Publisher gaze_direction_publisher;

    // it sent is_assessing message with the length
    ros::Publisher assessment_publisher;

    // Gaze tracking, absolute gaze direction
    Point3f gazeDirection0;
    Point3f gazeDirection1;

    // Gaze with respect to head rather than camera (for example if eyes are rolled up and the head is tilted or turned this will be stable)
    Point3f gazeDirection0_head;
    Point3f gazeDirection1_head;

    ros::Publisher detection_rate_publisher;

    int f_n;
    //the total number of frames displayed
    int frame_count;

    // the total number of sessions
    int num_frame_this_session;

    //the number of frames with a face detected
    int num_detected_frames;

    int total_frames;
    int reported_completion;

    double t0;

    cv::Mat captured_image;

    int64_t t_initial;
    double time_stamp;

    bool webcam;
    bool use_camera_plane_pose;
    bool video_output;

    vector<std::string> tracked_videos_output;
    vector<std::string> output_similarity_align;
    vector<std::string> output_au_files;
    vector<std::string> output_hog_align_files;
    vector<std::string> params_output_files;
    vector<std::string> gaze_output_files;

    // CLMTracker::CLM clm_model;
    vector<CLMTracker::CLMParameters> clm_parameters;

    // The modules that are being used for tracking
    vector<CLMTracker::CLM> clm_models;
    vector<bool> active_models;
    vector<FaceAnalysis::FaceAnalyser> face_analysers;

    std::ofstream hog_output_file;

    int num_hog_rows;
    int num_hog_cols;

    bool video_input;
    bool images_as_video;
    bool visualise_hog;
    bool verbose;

    float fx, fy;
    bool  fx_undefined;
    float cx, cy;
    bool  cx_undefined;

    // float global_detection_certainty = 0;

    bool init;

    geometry_msgs::Vector3 hfv_cf_msg;
    geometry_msgs::Vector3 headposition_cf_msg;

    // parameters needed for face recognition
    vector<Mat> faces_train;
    vector<int> labels_train;
    bool is_face_recognizer_set;
    Ptr<cv::face::FaceRecognizer> face_recognizer;
    int child_confidence_threshold;
    int parent_confidence_threshold;

    bool is_assessment;
    bool is_identification_assessment_done;
    bool is_target_screen_assessment_done;
    bool is_target_robot_assessment_done;
    bool is_target_human_assessment_done;
    bool is_target_other_assessment_done;
    float assessment_length; // in minutes
    bool is_assessing;
    double start_assessment_time;
    std::string parent_role;

    // Useful utility for creating directories for storing the output files
    void create_directory_from_file(std::string output_path);
    bool publishImage(cv::Mat &mat, const std::string encoding);

    // Extracting the following command line arguments -f, -fd, -op, -of, -ov (and possible ordered repetitions)
    void get_output_feature_params(vector<std::string> &similarity_aligned, bool &vid_output,
     vector<std::string> &gaze_files, vector<std::string> &hog_aligned_files,
     vector<std::string> &model_param_files, vector<std::string> &au_files,
     double &similarity_scale, int &similarity_size, bool &grayscale, bool &rigid,
     bool& verbose, vector<std::string> &arguments);

    void NonOverlappingDetections(const vector<CLMTracker::CLM>& clm_models,
      vector<cv::Rect_<double> >& face_detections);

    /**
    * Callback on the subscriber's topic.
    * @param msgIn an RGB image
    */
    void callback(const sensor_msgs::ImageConstPtr& msgIn);

    void parentRoleCallback(const std_msgs::String& msg);

    // get face image
    void retrieveFaceImage(cv::Mat img, const CLMTracker::CLM& clm_model, int & label, double & confidence, int model);

    void system_callback(const sar_core::SystemState::ConstPtr& msg);

public:
    ClmWrapper(std::string _name, std::string _loc);

    ~ClmWrapper() {};

    bool is_assessing_both_face();
    bool is_assessing_identification_done();
    bool is_assessing_screen_done();
    bool is_assessing_robot_done();
    bool is_assessing_human_done();
    bool is_assessing_other_done();
    void set_is_accessing();
    void send_assessment_message(int task, int state);
};
