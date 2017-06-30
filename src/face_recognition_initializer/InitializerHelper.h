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

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

#include <math.h>

#include <tf/transform_datatypes.h>

// call back on the mouse click event on the image
static void mouse_callback(int event, int x, int y, int flags, void* userdata);

class InitializerHelper
{
private:
    std::string face_recognizer_file_location;
    std::string face_assessment_file_location;

    bool is_get_data;
    bool is_to_train;
    bool is_assessment;
    bool is_assessing;
    bool is_child_assessment_done;
    bool is_mom_assessment_done;
    bool is_dad_assessment_done;
    bool is_using_mom_model;
    double assessment_length;
    clock_t start_assessment_time;
    int assessment_tracking_total_num;
    int assessment_label_total_num;
    string assessment_label_correct_answer;
    int assessment_tracking_correct_num;
    int assessment_label_correct_num;

    int assessment2_label_correct_num;

    // std::ofstream child_assessment_label_file;
    // std::ofstream child_assessment_tracking_file;
    // std::ofstream parent_assessment_label_file;
    // std::ofstream parent_assessment_tracking_file;

    std::ofstream mom_child_assessment_label_file;
    std::ofstream mom_child_assessment_tracking_file;
    std::ofstream mom_assessment_label_file;
    std::ofstream mom_assessment_tracking_file;
    std::ofstream dad_child_assessment_label_file;
    std::ofstream dad_child_assessment_tracking_file;
    std::ofstream dad_assessment_label_file;
    std::ofstream dad_assessment_tracking_file;

    std::string executable_location;

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;

    // publisher for the detected face images
    // image_transport::Publisher imagePublisher;

    //the total number of frames displayed
    int frame_count;
    //the number of frames with a face detected
    int num_detected_frames;
    int total_frames;
    int reported_completion;

    cv::Mat captured_image;

    CLMTracker::CLM clm_model;
    vector<CLMTracker::CLMParameters> clm_parameters;

    // The modules that are being used for tracking
    vector<CLMTracker::CLM> clm_models;
    vector<bool> active_models;

    double t0;

    bool video_input;
    bool images_as_video;

    // parameters needed for face recognition
    vector<Mat> mom_faces_train;
    vector<int> mom_labels_train;
    vector<Mat> dad_faces_train;
    vector<int> dad_labels_train;
    Ptr<cv::face::FaceRecognizer> face_recognizer; 
    Ptr<cv::face::FaceRecognizer> face_recognizer_mom;
    Ptr<cv::face::FaceRecognizer> face_recognizer_dad;
    bool is_train;
    bool is_model_trained;
    int train_stage;
    int num_train_samples;

    void publishImage(cv::Mat &mat);

    void NonOverlappingDetections(const vector<CLMTracker::CLM>& clm_models,
      vector<cv::Rect_<double> >& face_detections);

    /**
    * Callback on the subscriber's topic.
    * @param msgIn an RGB image
    */
    void callback(const sensor_msgs::ImageConstPtr& msgIn);

    // get face image
    void retrieveFaceImage(cv::Mat img, const CLMTracker::CLM& clm_model);

    string get_stage_task(int stage);

public:
    InitializerHelper(std::string _name, std::string _loc);
    ~InitializerHelper() {};
    bool is_training();
    bool is_training_done();
    bool is_trained();
    void stopTraining();
    void startNewTraining();
    void train();
    bool to_assess();
    bool is_assessment_done();
    void start_assessment();
    void switch_role();
};
