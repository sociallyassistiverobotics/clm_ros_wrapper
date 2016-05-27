///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, University of Cambridge,
// all rights reserved.
///////////////////////////////////////////////////////////////////////////////

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

class ClmWrapper
{
private:
    std::string name;
    std::string executable_location;

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;

    // publisher for the detected face images
    image_transport::Publisher imagePublisher;

    // can be called through the "clm_ros_wrapper/heads" topic
    ros::Publisher headsPublisher;

    int f_n;

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

    CLMTracker::CLM clm_model;
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

    bool init;

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

public:
    ClmWrapper(std::string _name, std::string _loc);
	

    ~ClmWrapper() {};
};
