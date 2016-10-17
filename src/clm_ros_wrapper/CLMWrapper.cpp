///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, University of Cambridge,
// all rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "CLMWrapper.h"

using namespace std;
using namespace cv;

using namespace boost::filesystem;

// Useful utility for creating directories for storing the output files
void ClmWrapper::create_directory_from_file(string output_path)
{
    // First get rid of the file
    auto p = path(path(output_path).parent_path());

    if(!p.empty() && !boost::filesystem::exists(p))
    {
        if(!boost::filesystem::create_directories(p))
        {
            ROS_ERROR("Failed to create the directory: %s", p.string().c_str());
        }
    }
}

bool ClmWrapper::publishImage(cv::Mat &mat, const std::string encoding)
{
    cv_bridge::CvImage msgOut;
    msgOut.encoding = encoding;
    msgOut.image    = mat;

    imagePublisher.publish(msgOut.toImageMsg());
    return true;
}

// Extracting the following command line arguments -f, -fd, -op, -of, -ov (and possible ordered repetitions)
void ClmWrapper::get_output_feature_params(vector<string> &output_similarity_aligned, bool &vid_output,
 vector<string> &output_gaze_files, vector<string> &output_hog_aligned_files,
 vector<string> &output_model_param_files, vector<string> &output_au_files,
 double &similarity_scale, int &similarity_size, bool &grayscale, bool &rigid,
 bool& verbose, vector<string> &arguments)
{
    output_similarity_aligned.clear();
    vid_output = false;
    output_hog_aligned_files.clear();
    output_model_param_files.clear();

    bool* valid = new bool[arguments.size()];

    for(size_t i = 0; i < arguments.size(); ++i)
    {
        valid[i] = true;
    }

    string input_root = "";
    string output_root = "";

    // First check if there is a root argument
    // (so that videos and outputs could be defined more easily)
    for(size_t i = 0; i < arguments.size(); ++i)
    {
        if (arguments[i].compare("-root") == 0)
        {
            input_root = arguments[i + 1];
            output_root = arguments[i + 1];
            i++;
        }
        if (arguments[i].compare("-inroot") == 0)
        {
            input_root = arguments[i + 1];
            i++;
        }
        if (arguments[i].compare("-outroot") == 0)
        {
            output_root = arguments[i + 1];
            i++;
        }
    }

    for(size_t i = 0; i < arguments.size(); ++i)
    {
        if (arguments[i].compare("-simalignvid") == 0)
        {
            output_similarity_aligned.push_back(output_root + arguments[i + 1]);
            create_directory_from_file(output_root + arguments[i + 1]);
            vid_output = true;
            valid[i] = false;
            valid[i+1] = false;
            i++;
        }
        else if (arguments[i].compare("-oaus") == 0)
        {
            output_au_files.push_back(output_root + arguments[i + 1]);
            create_directory_from_file(output_root + arguments[i + 1]);
            vid_output = true;
            valid[i] = false;
            valid[i+1] = false;
            i++;
        }
        else if (arguments[i].compare("-ogaze") == 0)
        {
            output_gaze_files.push_back(output_root + arguments[i + 1]);
            create_directory_from_file(output_root + arguments[i + 1]);
            valid[i] = false;
            valid[i + 1] = false;
            i++;
        }
        else if (arguments[i].compare("-simaligndir") == 0)
        {
            output_similarity_aligned.push_back(output_root + arguments[i + 1]);
            create_directory(output_root + arguments[i + 1]);
            vid_output = false;
            valid[i] = false;
            valid[i+1] = false;
            i++;
        }
        else if(arguments[i].compare("-hogalign") == 0)
        {
            output_hog_aligned_files.push_back(output_root + arguments[i + 1]);
            create_directory_from_file(output_root + arguments[i + 1]);
            valid[i] = false;
            valid[i+1] = false;
            i++;
        }
        else if(arguments[i].compare("-verbose") == 0)
        {
            verbose = true;
        }
        else if(arguments[i].compare("-oparams") == 0)
        {
            output_model_param_files.push_back(output_root + arguments[i + 1]);
            create_directory_from_file(output_root + arguments[i + 1]);
            valid[i] = false;
            valid[i+1] = false;
            i++;
        }
        else if(arguments[i].compare("-rigid") == 0)
        {
            rigid = true;
        }
        else if(arguments[i].compare("-g") == 0)
        {
            grayscale = true;
            valid[i] = false;
        }
        else if (arguments[i].compare("-simscale") == 0)
        {
            similarity_scale = stod(arguments[i + 1]);
            valid[i] = false;
            valid[i+1] = false;
            i++;
        }
        else if (arguments[i].compare("-simsize") == 0)
        {
            similarity_size = stoi(arguments[i + 1]);
            valid[i] = false;
            valid[i+1] = false;
            i++;
        }
        else if (arguments[i].compare("-help") == 0)
        {
            cout << "Output features are defined as: -simalign <outputfile>\n"; // Inform the user of how to use the program
        }
    }

    for(int i=arguments.size()-1; i >= 0; --i)
    {
        if(!valid[i])
        {
            arguments.erase(arguments.begin()+i);
        }
    }
}

void ClmWrapper::NonOverlappingDetections(const vector<CLMTracker::CLM>& clm_models, vector<Rect_<double> >& face_detections)
{
    // Go over the model and eliminate detections that are not informative (there already is a tracker there)
    for(size_t model = 0; model < clm_models.size(); ++model)
    {
        // See if the detections intersect
        Rect_<double> model_rect = clm_models[model].GetBoundingBox();

        for(int detection = face_detections.size()-1; detection >=0; --detection)
        {
            double intersection_area = (model_rect & face_detections[detection]).area();
            double union_area = model_rect.area() + face_detections[detection].area() - 2 * intersection_area;

            // If the model is already tracking what we're detecting ignore the detection, this is determined by amount of overlap
            if( intersection_area/union_area > 0.5)
            {
                face_detections.erase(face_detections.begin() + detection);
            }
        }
    }
}

/**
* Callback on the subscriber's topic.
* @param msgIn an RGB image
*/

void ClmWrapper::callback(const sensor_msgs::ImageConstPtr& msgIn)
{
    // Convert the ROS image to OpenCV image format
    // BUG : For CLM, OpenCV 3.* is needed, but cv_bridge segfaults with openCV 3.0
    // when asked to convert images with BGR encoding. The solution has been to convert them
    // without a specified encoding (I think it falls back to RGB) and then manually convert
    // them to BGR
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msgIn);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image.clone(), captured_image, CV_BGR2RGB);

    // cv::imshow("output", captured_image);
    // cv::waitKey(5);
    // return;

    // Stuff to run on the first frame
    if (init == true )
    {
        // Approximate camera parameters TODO : read them from the parameter server
        // If cx (optical axis centre) is undefined will use
        // the image size/2 as an estimate (i.e. the center of the image)
        if(cx_undefined)
        {
            cx = captured_image.cols / 2.0f;
            cy = captured_image.rows / 2.0f;
        }
        // Use a rough guess-timate of focal length
        if (fx_undefined)
        {
            fx = 500 * (captured_image.cols / 640.0);
            fy = 500 * (captured_image.rows / 480.0);

            fx = (fx + fy) / 2.0;
            fy = fx;
        }

        init = false;
    }

    typedef clm_ros_wrapper::ClmHeads ClmHeadsMsg;
    typedef clm_ros_wrapper::ClmHead ClmHeadMsg;
    typedef clm_ros_wrapper::ClmEyeGaze ClmEyeGazeMsg;
    typedef clm_ros_wrapper::ClmFacialActionUnit ClmFacialActionUnitMsg;
    typedef clm_ros_wrapper::GazeDirection GazeDirectionMsg;

    // Set the timestamp
    int64 curr_time = cv::getTickCount();
    time_stamp = (double(curr_time - t_initial) / cv::getTickFrequency());

    // Reading the images
    Mat_<float> depth_image;
    Mat_<uchar> grayscale_image;
    Mat disp_image = captured_image.clone();

    if(captured_image.channels() == 3) cvtColor(captured_image, grayscale_image, CV_BGR2GRAY);
    else                               grayscale_image = captured_image.clone();

    // where the current face detections are stored
    vector<Rect_<double> > face_detections;

    bool all_models_active = true;
    for(unsigned int model = 0; model < clm_models.size(); ++model)
    {
        if(!active_models[model])
        {
            all_models_active = false;
        }
    }

    // eric: we do the frame detection earlier in the code base
    // Get the detections (every 8th frame and when there are free models available for tracking)
    if(frame_count % 4 == 0 && !all_models_active) //(frame_count % 4 == 0 && !all_models_active)
    {
        if(clm_parameters[0].curr_face_detector == CLMTracker::CLMParameters::HOG_SVM_DETECTOR)
        {
            vector<double> confidences;
            CLMTracker::DetectFacesHOG(face_detections, grayscale_image, clm_models[0].face_detector_HOG, confidences);
        }
        else
        {
            CLMTracker::DetectFaces(face_detections, grayscale_image, clm_models[0].face_detector_HAAR);
        }

    }

    // Keep only non overlapping detections (also convert to a concurrent vector
    NonOverlappingDetections(clm_models, face_detections);

    vector<tbb::atomic<bool> > face_detections_used(face_detections.size());

    ClmHeadsMsg ros_heads_msg;

    // Go through every model and update the tracking TODO pull out as a separate parallel/non-parallel method
    tbb::parallel_for(0, (int)clm_models.size(), [&](int model)
    {
        bool detection_success = false;

        // If the current model has failed more than 4 times in a row, remove it
        if(clm_models[model].failures_in_a_row > 4)
        {
            active_models[model] = false;
            clm_models[model].Reset();
        }

        // If the model is inactive reactivate it with new detections
        if(!active_models[model])
        {
            for(size_t detection_ind = 0; detection_ind < face_detections.size(); ++detection_ind)
            {
                // if it was not taken by another tracker take it
                // (if it is false swap it to true and enter detection, this makes it parallel safe)
                if(face_detections_used[detection_ind].compare_and_swap(true, false) == false)
                {

                    // Reinitialise the model
                    clm_models[model].Reset();

                    // This ensures that a wider window is used for the initial landmark localisation
                    clm_models[model].detection_success = false;
                    //IGNORE STANDALONE IMAGES
                    if(video_input || images_as_video)
                    {
                        detection_success = CLMTracker::DetectLandmarksInVideo(grayscale_image, depth_image, face_detections[detection_ind],
                         clm_models[model], clm_parameters[model]);
                    }
                    else
                    {
                        ROS_FATAL_STREAM("Standalone images cannot be used in this release");
                        //detection_success = CLMTracker::DetectLandmarksInImage(grayscale_image, clm_model, clm_params);
                    }
                    // This activates the model
                    active_models[model] = true;

                    // break out of the loop as the tracker has been reinitialised
                    break;
                }
            }
        }

        else
        {
            //IGNORE STANDALONE IMAGES
            if(video_input || images_as_video)
            {
                detection_success = CLMTracker::DetectLandmarksInVideo(grayscale_image, depth_image, clm_models[model], clm_parameters[model]);
            }
            else
            {
                ROS_FATAL_STREAM("Standalone images cannot be used in this release");
                //detection_success = CLMTracker::DetectLandmarksInImage(grayscale_image, clm_model, clm_params);
            }

            // Gaze tracking, absolute gaze direction
            //Point3f gazeDirection0;
            //Point3f gazeDirection1;

            // Gaze with respect to head rather than camera (for example if eyes are rolled up and the head is tilted or turned this will be stable)
            //Point3f gazeDirection0_head;
            //Point3f gazeDirection1_head;

            if (clm_parameters[model].track_gaze && detection_success)
            {
                FaceAnalysis::EstimateGaze(clm_models[model], gazeDirection0, gazeDirection0_head, fx, fy, cx, cy, true); //left
                FaceAnalysis::EstimateGaze(clm_models[model], gazeDirection1, gazeDirection1_head, fx, fy, cx, cy, false); //right
            }

            // Do face alignment
            //Mat sim_warped_img;
            //Mat_<double> hog_descriptor;

            // But only if needed in output
            // std::cout << "not empty: output_similarity_align " << !output_similarity_align.empty() << " is open: hog_output_file " <<
            //              hog_output_file.is_open() << " not empty: output_au_files " << !output_au_files.empty() << std::endl;
            // if(!output_similarity_align.empty() || hog_output_file.is_open() || !output_au_files.empty()) START CHECK
            // {
            // face_analysers[model].AddNextFrame(captured_image, clm_models[model], time_stamp, webcam, !clm_parameters[model].quiet_mode);
            // face_analysers[model].GetLatestAlignedFace(sim_warped_img);

            //FaceAnalysis::AlignFaceMask(sim_warped_img, captured_image, clm_model, triangulation, rigid, sim_scale, sim_size, sim_size);
            // if(!clm_parameters[model].quiet_mode)
            // {
            //     //cv::imshow("sim_warp", sim_warped_img);
            //     //cv::waitKey(1);
            // }

            // if(hog_output_file.is_open())
            // {
            //     FaceAnalysis::Extract_FHOG_descriptor(hog_descriptor, sim_warped_img, num_hog_rows, num_hog_cols);

            //     if(visualise_hog && !clm_parameters[model].quiet_mode)
            //     {
            //         Mat_<double> hog_descriptor_vis;
            //         FaceAnalysis::Visualise_FHOG(hog_descriptor, num_hog_rows, num_hog_cols, hog_descriptor_vis);
            //         cv::imshow("hog", hog_descriptor_vis);
            //     }
            // }
            // } END CHECK

            // Work out the pose of the head from the tracked model
            Vec6d pose_estimate_CLM;
            if(use_camera_plane_pose)
            {
                pose_estimate_CLM = CLMTracker::GetCorrectedPoseWorld(clm_models[model], fx, fy, cx, cy);
            }
            else
            {
                pose_estimate_CLM = CLMTracker::GetCorrectedPoseCamera(clm_models[model], fx, fy, cx, cy);
            }

            // e: don't think we're using hog
            //if(hog_output_file.is_open())
            //{
            //  output_HOG_frame(&hog_output_file, detection_success, hog_descriptor, num_hog_rows, num_hog_cols);
            //}

            double confidence = 0.5 * (1 - clm_model.detection_certainty);

            ClmHeadMsg ros_head_msg;
            auto & ros_eyegazes_msg = ros_head_msg.eyegazes;
            auto & ros_aus_msg = ros_head_msg.aus;

            ros_head_msg.detection_success = static_cast<uint8_t>( detection_success );
            ros_head_msg.detection_confidence = static_cast<float>( confidence );
            ros_head_msg.time_stamp = static_cast<float>( time_stamp );

            // package head pose message
            ros_head_msg.headpose.x = static_cast<float>( pose_estimate_CLM[0] );
            ros_head_msg.headpose.y = static_cast<float>( pose_estimate_CLM[1] );
            ros_head_msg.headpose.z = static_cast<float>( pose_estimate_CLM[2] );
            ros_head_msg.headpose.pitch = static_cast<float>( pose_estimate_CLM[3] );
            ros_head_msg.headpose.yaw = static_cast<float>( pose_estimate_CLM[4] );
            ros_head_msg.headpose.roll = static_cast<float>( pose_estimate_CLM[5] );

            // using the matrix we found in clm_utils.cpp
            Matx33d headRotationMatrixCLM_cf = CLMTracker::Euler2RotationMatrix(Vec3d(pose_estimate_CLM[3], pose_estimate_CLM[4], pose_estimate_CLM[5]));

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
            tfScalar array_from_rotation_matrix[12];
            for (int i = 0; i<3 ; i++)
            {
                for (int j = 0;j<3;j++)
                {
                    array_from_rotation_matrix[4*i + j] = headRotationMatrixCLM_cf(j,i);
                    //cout << endl << headRotationMatrixCLM_cf(j,i) <<  " " << i <<" " << j << endl;
                }
            }

            tf::Matrix3x3 head_rotation_cf;
            head_rotation_cf.setFromOpenGLSubMatrix(array_from_rotation_matrix);

            // head fixation vector is equal to -1 * the third column of the head rotation matrix
            tf::Vector3 hfv_cf = head_rotation_cf * tf::Vector3(0, 0, -1);

            //converting to type geometry_msgs::Vector3 and overwriting hfv_cf_msg
            tf::vector3TFToMsg(hfv_cf, hfv_cf_msg);

            tf::Vector3 headposition_cf = tf::Vector3(ros_head_msg.headpose.x, ros_head_msg.headpose.y , ros_head_msg.headpose.z);
            //tf::Vector3 headposition_cf = tf::Vector3(0, -60 , 450);

            //converting to type geometry_msgs::Vector3 and overwriting headposition_cf_msg
            tf::vector3TFToMsg(headposition_cf, headposition_cf_msg);

            std::vector<Point3f> gazeDirections = {gazeDirection0, gazeDirection1};// left, right
            std::vector<Point3f> gazeDirections_head = {gazeDirection0_head, gazeDirection1_head};

            //new gaze direction message
            GazeDirectionMsg ros_gaze_direction_msg;
            geometry_msgs::Vector3 lefe_gaze_direction_cf;
            lefe_gaze_direction_cf.x = gazeDirection0.x;
            lefe_gaze_direction_cf.y = gazeDirection0.y;
            lefe_gaze_direction_cf.z = gazeDirection0.z;
            geometry_msgs::Vector3 right_gaze_direction_cf;
            right_gaze_direction_cf.x = gazeDirection1.x;
            right_gaze_direction_cf.y = gazeDirection1.y;
            right_gaze_direction_cf.z = gazeDirection1.z;

            ros_gaze_direction_msg.left_gaze_diection = lefe_gaze_direction_cf;
            ros_gaze_direction_msg.right_gaze_diection = right_gaze_direction_cf;
            gaze_direction_publisher.publish(ros_gaze_direction_msg);

            for (size_t p = 0; p < gazeDirections_head.size(); p++)
            {
                ClmEyeGazeMsg ros_eyegaze_msg;
                ros_eyegaze_msg.eye_id = p;// 0: left, 1: right
                ros_eyegaze_msg.gaze_direction_cameraref_x = static_cast<float>( gazeDirections[p].x );
                ros_eyegaze_msg.gaze_direction_cameraref_y = static_cast<float>( gazeDirections[p].y );
                ros_eyegaze_msg.gaze_direction_cameraref_z = static_cast<float>( gazeDirections[p].z );
                ros_eyegaze_msg.gaze_direction_headref_x = static_cast<float>( gazeDirections_head[p].x ); //lateral gaze
                ros_eyegaze_msg.gaze_direction_headref_y = static_cast<float>( gazeDirections_head[p].y );
                ros_eyegaze_msg.gaze_direction_headref_z = static_cast<float>( gazeDirections_head[p].z );
                ros_eyegazes_msg.emplace_back( std::move( ros_eyegaze_msg ) );
                eye_gaze_publisher.publish(ros_eyegaze_msg);
            }
            //eyesMsgPublisher.publish(ros_eyegaze_msg);
            //AU01_r, AU04_r, AU06_r, AU10_r, AU12_r, AU14_r, AU17_r, AU25_r, AU02_r, AU05_r,
            //AU09_r, AU15_r, AU20_r, AU26_r, AU12_c, AU23_c, AU28_c, AU04_c, AU15_c, AU45_c

            // package facial action unit message
            std::vector<int> au_r_handles = {1, 4, 6, 10, 12, 14, 17, 25, 2, 5, 9, 15, 20, 26};
            std::vector<int> au_c_handles = {12, 23, 28, 4, 15, 45};

            std::vector<pair<string, double>> aus_reg = face_analysers[model].GetCurrentAUsReg();

            if(aus_reg.size() == 0)
            {
                for(size_t p = 0; p < face_analysers[model].GetAURegNames().size(); p++)
                {
                    ClmFacialActionUnitMsg ros_au_msg;
                    ros_au_msg.type = static_cast<uint8_t>( au_r_handles[p] );
                    ros_au_msg.value = 0;
                    ros_au_msg.prediction_method = 0;
                    ros_aus_msg.emplace_back( std::move( ros_au_msg ) );
                }
            }
            else
            {
                for(size_t p = 0; p < aus_reg.size(); p++)
                {
                    ClmFacialActionUnitMsg ros_au_msg;
                    ros_au_msg.type = static_cast<uint8_t>( au_r_handles[p] );
                    ros_au_msg.value = static_cast<float>( aus_reg[p].second );
                    ros_au_msg.prediction_method = 0;
                    ros_aus_msg.emplace_back( std::move( ros_au_msg ) );
                }
            }

            std::vector<pair<string, double>> aus_class = face_analysers[model].GetCurrentAUsClass();

            if(aus_class.size() == 0)
            {
                for(size_t p = 0; p < face_analysers[model].GetAUClassNames().size(); p++)
                {
                    ClmFacialActionUnitMsg ros_au_msg;
                    ros_au_msg.type = static_cast<uint8_t>( au_c_handles[p] );
                    ros_au_msg.value = 0;
                    ros_au_msg.prediction_method = 1;
                    ros_aus_msg.emplace_back( std::move( ros_au_msg ) );
                }
            }
            else
            {
                for(size_t p = 0; p < aus_class.size(); p++)
                {
                    ClmFacialActionUnitMsg ros_au_msg;
                    ros_au_msg.type = static_cast<uint8_t>( au_c_handles[p] );
                    ros_au_msg.value = static_cast<float>( aus_class[p].second );
                    ros_au_msg.prediction_method = 1;
                    ros_aus_msg.emplace_back( std::move( ros_au_msg ) );
                }
            }

            ros_heads_msg.heads.emplace_back( std::move( ros_head_msg ) );

        }

    });

    std_msgs::String rate;

    std::stringstream ss;
    ss << " " << ((float) num_detected_frames)/frame_count << " ";
    rate.data = ss.str();

    detection_rate_publisher.publish(rate);

    headsPublisher.publish( ros_heads_msg );

    // used to check if a face is detected in this iteration
    int faceDetected = 0;
    double detection_certainty = 0.0;

    // Go through every model and visualise the results
    for(size_t model = 0; model < clm_models.size(); ++model)
    {
        // Draw the facial landmarks on the face and the bounding box around it
        // if tracking is successful and initialized
        detection_certainty = clm_models[model].detection_certainty;
        global_detection_certainty = detection_certainty;
        double visualisation_boundary = -0.1;
        // Only draw if the reliability is reasonable, the value is slightly ad-hoc
        if(detection_certainty < visualisation_boundary)
        {
            faceDetected = 1;

            CLMTracker::Draw(disp_image, clm_models[model]);
            if(detection_certainty > 1)     detection_certainty =  1;
            if(detection_certainty < -1)    detection_certainty = -1;

            detection_certainty = (detection_certainty + 1)/(visualisation_boundary +1);

            global_detection_certainty = detection_certainty;

            // A rough heuristic for box around the face width
            int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

            // // Work out the pose of the head from the tracked model
            Vec6d pose_estimate_CLM = CLMTracker::GetCorrectedPoseWorld(clm_models[model], fx, fy, cx, cy);

            // Draw it in reddish if uncertain, blueish if certain
            CLMTracker::DrawBox(disp_image, pose_estimate_CLM, Scalar((1-detection_certainty)*255.0,0,

                detection_certainty*255), thickness, fx, fy, cx, cy);

            FaceAnalysis::DrawGaze(disp_image, clm_models[model], gazeDirection0, gazeDirection1, fx, fy, cx, cy);

            // displaying tracking certainty
            char certainty_C[255];
            sprintf(certainty_C, "%f", 1-detection_certainty);
            string certainty_st("Certainty: ");
            certainty_st += certainty_C;
            cv::Point certainty_pos;
            vector<std::pair<Point,Point>> certainty_pos_vec = CLMTracker::CalculateBox(pose_estimate_CLM, fx, fy, cx, cy);
            if (12 == certainty_pos_vec.size()) {
                certainty_pos = certainty_pos_vec[10].second; // certainty on the head
            } else {
                certainty_pos = cv::Point(10, 100); // default position on the left top corner
            }
            cv::putText(disp_image, certainty_st, certainty_pos, CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0));

            // cout << model << " "<< fx << " " << fy << " " << cx << " " << cy << " " << detection_certainty << " " << thickness
            // << " " << pose_estimate_CLM[0] << " " << pose_estimate_CLM[1] << " " << pose_estimate_CLM[2]
            // << " " << pose_estimate_CLM[3] << " " << pose_estimate_CLM[4] << " " << pose_estimate_CLM[5] << endl;
        }
    }
    // Write out the framerate on the image before displaying it
    char fpsC[255];
    int fps_tracker;

    if(frame_count % 10 == 0)
    {
       double t1 = cv::getTickCount();
       fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
       t0 = t1;
    }

    sprintf(fpsC, "%d", (int)fps_tracker);

    string fpsSt("FPS:");
    fpsSt += fpsC;

    cv::putText(disp_image, fpsSt, cv::Point(10,20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0));

    int num_active_models = 0;

    for( size_t active_model = 0; active_model < active_models.size(); active_model++)
    {
        if(active_models[active_model])
        {
            num_active_models++;
        }
    }

    char active_m_C[255];
    sprintf(active_m_C, "%d", num_active_models);
    string active_models_st("Active models:");
    active_models_st += active_m_C;
    cv::putText(disp_image, active_models_st, cv::Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0));

    if (faceDetected)
    {
        num_detected_frames++;
        // feeding disp_image into the cv pointer's image
        // only if a face is detected
        publishImage(disp_image,"bgr8");
    }
    else
    {
        publishImage(captured_image, "bgr8");\

        //creating dummy vectors to make hfv_publisher and head_position_publisher publish
        //some vector regardless of whether something is detected
        tf::Vector3 no_face_detection_head_vector = tf::Vector3(0,0,0);
        tf::Vector3 no_face_detection_head_position = tf::Vector3(0, 0, 0);

        tf::vector3TFToMsg(no_face_detection_head_position, headposition_cf_msg);
        tf::vector3TFToMsg(no_face_detection_head_vector, hfv_cf_msg);
    }
     //publishing the image usign the cv pointer

    // e: don't need to work out framerate
    // Work out the framerate
    //if(frame_count % 10 == 0)
    //{
    //  double t1 = cv::getTickCount();
    //  fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
    //  t0 = t1;
    //}


    // if(!clm_parameters[0].quiet_mode)
    // {
    //      namedWindow("tracking_result",1);
    //      cv::imshow("tracking_result", disp_image);
    // }

    // head fixation vector and the head position publisher (both in camera frame)
    hfv_publisher.publish(hfv_cf_msg);

    clm_ros_wrapper::VectorWithCertainty headpos_certainty_msg;
    headpos_certainty_msg.position = headposition_cf_msg;

    //global_detection_certainty between 0 and 1 and decreases as the detection gets certain
    headpos_certainty_msg.certainty = 1- global_detection_certainty;

    head_position_publisher.publish(headpos_certainty_msg);

    // e: don't need to work out framerate
    // Work out the framerate
    //if(frame_count % 10 == 0)
    //{
    //  double t1 = cv::getTickCount();
    //  fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
    //  t0 = t1;
    //}


    // if(!clm_parameters[0].quiet_mode)
    // {
    //      namedWindow("tracking_result",1);
    //      cv::imshow("tracking_result", disp_image);
    // }

       // e: removing key press code
       // detect key presses
       //char character_press = cv::waitKey(1);

       //// restart the trackers
       //if(character_press == 'r')
       //{
       //  for(size_t i=0; i < clm_models.size(); ++i)
       //  {
       //    clm_models[i].Reset();
       //    active_models[i] = false;
       //  }
       //}
       //// quit the application
       //else if(character_press=='q')
       //{
       //  return(0);
       //}


       // Update the frame count
    frame_count++;

    if(total_frames != -1)
    {
        if((double)frame_count/(double)total_frames >= reported_completion / 10.0)
        {
            //cout << reported_completion * 10 << "%";
            reported_completion = reported_completion + 1;
        }
    }
     //}

     // e: not needed for our purposes
     //if(total_frames != -1)
     //{
     //  cout << endl;
     //}

     //
     //curr_img = -1;

     //// Reset the model, for the next video
     //for(size_t model=0; model < clm_models.size(); ++model)
     //{
     //  clm_models[model].Reset();
     //  active_models[model] = false;
     //}


     //// break out of the loop if done with all the files (or using a webcam)
     //if(f_n == files.size() -1 || files.empty())
     //{
     //  done = true;
     //}

}

ClmWrapper::ClmWrapper(string _name, string _loc) : name(_name), executable_location(_loc), imageTransport(nodeHandle)
{
    ROS_INFO("Called constructor...");

    string _cam = "/usb_cam";
    nodeHandle.getParam("cam", _cam);
    // raw camera image
    imageSubscriber = imageTransport.subscribe(_cam+"/image_raw",1,&ClmWrapper::callback, this);

    //current not used
    headsPublisher  = nodeHandle.advertise<clm_ros_wrapper::ClmHeads>(_name+"/heads",1);

    // publisher for the image when a face is detected
    imagePublisher = imageTransport.advertise(_name+"/face_image", 1);

    // publishing head direction in cf
    hfv_publisher = nodeHandle.advertise<geometry_msgs::Vector3>(_name+"/head_vector", 1);

    // publishing head position in the camera frame
    head_position_publisher = nodeHandle.advertise<clm_ros_wrapper::VectorWithCertainty>(_name+"/head_position", 1);

    detection_rate_publisher = nodeHandle.advertise<std_msgs::String>(_name+"/detection_rate", 1);

    // publishing eye gaze
    eye_gaze_publisher = nodeHandle.advertise<clm_ros_wrapper::ClmEyeGaze>(_name+"/eye_gaze", 1);

    gaze_direction_publisher = nodeHandle.advertise<clm_ros_wrapper::GazeDirection>(_name+"/gaze_direction", 1);

    init = true;

    // code to start a window

    //cv::namedWindow("output", cv::WINDOW_NORMAL);
    //cv::startWindowThread();
    //cv::moveWindow("output", 1050, 50);

    typedef clm_ros_wrapper::ClmHeads ClmHeadsMsg;
    typedef clm_ros_wrapper::ClmHead ClmHeadMsg;
    typedef clm_ros_wrapper::ClmEyeGaze ClmEyeGazeMsg;
    typedef clm_ros_wrapper::ClmFacialActionUnit ClmFacialActionUnitMsg;

    vector<string> arguments;
    arguments.push_back(executable_location);
    // Some initial parameters that can be overriden from command line
    vector<string> files, depth_directories, pose_output_files, landmark_output_files, landmark_3D_output_files;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.

    // ERICNOTE: this stuff doesn't look too useful - don't know what HOG is though
    CLMTracker::CLMParameters clm_params(arguments);
    clm_params.use_face_template = true;
    clm_params.reinit_video_every = -1;   // This is to avoid the model to try re-initialising itself
    clm_params.curr_face_detector = CLMTracker::CLMParameters::HOG_SVM_DETECTOR;

    // TODO a command line argument
    clm_params.track_gaze = true;

    //vector<CLMTracker::CLMParameters> clm_parameters;
    clm_parameters.push_back(clm_params);

    // Get the input output file parameters

    // ERICNOTE: the following section deals with image files
    // Indicates that rotation should be with respect to camera plane or with respect to camera
    video_input = true;
    verbose = true;
    images_as_video = false;
    webcam = false;
    use_camera_plane_pose = false;

    vector<vector<string> > input_image_files;

    //ENDNOTE

    fx = 0, fy = 0;
    cx = 0, cy = 0;

    cx_undefined = true;
    fx_undefined = true;


    //vector<string> output_similarity_align;
    //vector<string> output_au_files;
    //vector<string> output_hog_align_files;
    //vector<string> params_output_files;
    //vector<string> gaze_output_files;

    double sim_scale = 0.7;
    int sim_size = 112;
    bool grayscale = false;
    video_output = false;
    bool rigid = false;
    get_output_feature_params(output_similarity_align, video_output, gaze_output_files,
      output_hog_align_files, params_output_files, output_au_files,
      sim_scale, sim_size, grayscale, rigid, verbose, arguments);

    // Used for image masking - triangulation code

    Mat_<int> triangulation;
    string tri_loc  = "";
    string tri_name = "model/tris_68_full.txt";
    path loc = path(arguments[0]).parent_path() / tri_name.c_str();
    tri_loc = loc.string();

    if(boost::filesystem::exists(path(tri_name.c_str())))
    {
        std::ifstream triangulation_file(tri_name.c_str());
        CLMTracker::ReadMat(triangulation_file, triangulation);
        tri_loc = tri_name.c_str();
        ROS_INFO("[ClmWrapper] Triangulation file correctly read.");
    }
    else
    {
        path loc = path(arguments[0]).parent_path() / tri_name.c_str();
        tri_loc = loc.string();
        ROS_INFO("[ClmWrapper] Triangulation file: %s",tri_loc.c_str());

        if(exists(loc))
        {
            std::ifstream triangulation_file(loc.string());
            CLMTracker::ReadMat(triangulation_file, triangulation);
            ROS_INFO("[ClmWrapper] Triangulation file correctly read.");
        }
        else
        {
            ROS_ERROR("[ClmWrapper] Could not find triangulation files (i.e. %s), exiting.", tri_name.c_str());
            return;
        }
    }

    // If multiple video files are tracked, use this to indicate if we are done
    bool done = false;
    f_n = -1;
    int curr_img = -1;
    string au_loc  = "";
    string au_name = "AU_predictors/AU_all_best.txt";

    if (boost::filesystem::exists(path(au_name.c_str())))
    {
        au_loc = au_name;
    }
    else
    {
        path loc = path(arguments[0]).parent_path() / au_name.c_str();

        if(exists(loc))
        {
            au_loc = loc.string();
        }
        else
        {
            ROS_ERROR("[ClmWrapper] Could not find AU prediction files (i.e. %s), exiting.", au_name.c_str());
            return;
        }
    }

    // Creating a  face analyser that will be used for AU extraction
    FaceAnalysis::FaceAnalyser face_analyser(vector<Vec3d>(), 0.7, 112, 112, au_loc, tri_loc);

    //// The modules that are being used for tracking
    //vector<CLMTracker::CLM> clm_models;
    //vector<bool> active_models;
    //vector<FaceAnalysis::FaceAnalyser> face_analysers;

    int num_faces_max = 2;
    CLMTracker::CLM clm_model2(clm_parameters[0].model_location);
    //clm_model = clm_model2; //cmhuang: problematic...
    clm_model2.face_detector_HAAR.load(clm_parameters[0].face_detector_location);
    clm_model2.face_detector_location = clm_parameters[0].face_detector_location;

    // Will warp to scaled mean shape
    Mat_<double> similarity_normalised_shape = clm_model2.pdm.mean_shape * sim_scale;
    // Discard the z component
    similarity_normalised_shape = similarity_normalised_shape(Rect(0, 0, 1, 2*similarity_normalised_shape.rows/3)).clone();

    clm_models.reserve(num_faces_max);

    clm_models.push_back(clm_model2);
    active_models.push_back(false);
    face_analysers.push_back(face_analyser);

    for (int i = 1; i < num_faces_max; ++i)
    {
      clm_models.push_back(clm_model2);
      active_models.push_back(false);
      clm_parameters.push_back(clm_params);
      face_analysers.push_back(face_analyser);
    }

    string current_file;

    int total_frames = -1;
    int reported_completion = 0;

    double fps_vid_in = -1.0;

    webcam = true;

    frame_count = 0;
    num_detected_frames = 0;

        // This is useful for a second pass run (if want AU predictions)
    vector<Vec6d> params_global_video;
    vector<bool> successes_video;
    vector<Mat_<double>> params_local_video;
    vector<Mat_<double>> detected_landmarks_video;

        // Use for timestamping if using a webcam
    t_initial = cv::getTickCount();

    visualise_hog = verbose;
        // Timestamp in seconds of current processing
    time_stamp = 0;

    // assinging the default values of the dummy vectors
    tf::Vector3 no_face_detection_head_vector = tf::Vector3(0,0,0);
    tf::Vector3 no_face_detection_head_position = tf::Vector3(0, 0, 0);

    tf::vector3TFToMsg(no_face_detection_head_position, headposition_cf_msg);
    tf::vector3TFToMsg(no_face_detection_head_vector, hfv_cf_msg);
};
