///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, University of Cambridge,
// all rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "InitializerHelper.h"
#include <ctime>

using namespace std;
using namespace cv;

using namespace boost::filesystem;

vector<string> const tasks = {"STRAIGHT FORWARD", 
                              "TURN LEFT", 
                              "TURN RIGHT", 
                              "TURN UP", 
                              "TURN DOWN", 
                              "LEAN LEFT", 
                              "LEAN RIGHT", 
                              "TOP LEFT CORNER OF SCREEN", 
                              "TOP RIGHT CORNER OF SCREEN",
                              "BOTTOM LEFT CORNER OF SCREEN",
                              "BOTTOM RIGHT CORNER OF SCREEN",
                              "CENTER OF SCREEN",
                              "ROBOT",
                              "PARENT/CHILD"};
int static const num_stages = tasks.size();
int static const num_role = 3;


static void mouse_callback(int event, int x, int y, int flags, void* userdata)
{
    InitializerHelper * helper = (InitializerHelper *)userdata;
    if (event == EVENT_LBUTTONUP)
    {
        if (helper->is_training()) {
            helper->stopTraining();
        } else if (helper->is_training_done()) {
            if (helper->is_trained()) {
                if (helper->to_assess() && !helper->is_assessment_done()) {
                    helper->start_assessment();
                } else {
                    helper->switch_role();
                }
            } else {
                helper->train();
            }
        } else {
            helper->startNewTraining();
        }
    }
}

bool InitializerHelper::is_training()
{
    return is_train;
}

bool InitializerHelper::is_training_done()
{
    return !is_train && num_stages * num_role == train_stage;
}

bool InitializerHelper::is_trained()
{
    return is_model_trained;
}

void InitializerHelper::stopTraining()
{
    is_train = false;
    train_stage++;
    num_train_samples = 0;
}

void InitializerHelper::startNewTraining()
{
    is_train = true;
}

bool InitializerHelper::to_assess()
{
    return is_assessment;
}

bool InitializerHelper::is_assessment_done()
{
    return is_child_assessment_done && is_mom_assessment_done && is_dad_assessment_done;
}

void InitializerHelper::start_assessment()
{
    assessment_tracking_total_num = 0;
    assessment_tracking_correct_num = 0;
    assessment_label_total_num = 0;
    assessment_label_correct_num = 0;
    assessment2_label_correct_num = 0;

    if (!is_child_assessment_done) {
        assessment_label_correct_answer = "CHILD";
    } else if (!is_mom_assessment_done) {
        assessment_label_correct_answer = "MOM";
    } else if (!is_dad_assessment_done) {
        switch_role();
        assessment_label_correct_answer = "DAD";
    }

    is_assessing = true;
    start_assessment_time = std::clock();
}

void InitializerHelper::publishImage(cv::Mat &mat)
{
    imshow("Face Recognition Training", mat);
    waitKey(1);
}

void InitializerHelper::train()
{
    string location = face_recognizer_file_location + "train_images/";

    for(auto & folder : boost::make_iterator_range(directory_iterator(location), {})) {
        string parent_path = folder.path().parent_path().string();
        string current_path = folder.path().string();
        int label = std::stoi(current_path.substr(parent_path.length() + 1, current_path.length()));
        if (label <= num_stages) {
            for (auto & image : boost::make_iterator_range(directory_iterator(folder), {})) {
                mom_faces_train.push_back(cv::imread(image.path().string(), CV_LOAD_IMAGE_GRAYSCALE));
                mom_labels_train.push_back(label);
            }
        } else if (label <= 2 * num_stages) {
            for (auto & image : boost::make_iterator_range(directory_iterator(folder), {})) {
                dad_faces_train.push_back(cv::imread(image.path().string(), CV_LOAD_IMAGE_GRAYSCALE));
                dad_labels_train.push_back(label);
            }            
        } else {
            for (auto & image : boost::make_iterator_range(directory_iterator(folder), {})) {
                mom_faces_train.push_back(cv::imread(image.path().string(), CV_LOAD_IMAGE_GRAYSCALE));
                mom_labels_train.push_back(label);
                dad_faces_train.push_back(cv::imread(image.path().string(), CV_LOAD_IMAGE_GRAYSCALE));
                dad_labels_train.push_back(label);                
            }
        }
    }
    face_recognizer_mom->train(mom_faces_train, mom_labels_train);
    face_recognizer_mom->save(face_recognizer_file_location + "face_recognizer_model_mom.xml");

    face_recognizer_dad->train(dad_faces_train, dad_labels_train);
    face_recognizer_dad->save(face_recognizer_file_location + "face_recognizer_model_dad.xml");

    is_model_trained = true;
}

void InitializerHelper::NonOverlappingDetections(const vector<CLMTracker::CLM>& clm_models, vector<Rect_<double> >& face_detections)
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

void InitializerHelper::callback(const sensor_msgs::ImageConstPtr& msgIn)
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
    if(!all_models_active && frame_count % 3 == 0) //(frame_count % 4 == 0 && !all_models_active)
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

    // Go through every model and update the tracking TODO pull out as a separate parallel/non-parallel method
    tbb::parallel_for(0, (int)clm_models.size(), [&](int model)
    {
        // If the current model has failed more than 4 times in a row, remove it
        if(clm_models[model].failures_in_a_row > 4)
        {
            active_models[model] = false;
            clm_models[model].Reset();
        }

        // If the model is inactive reactivate it with new detections
        if(!active_models[model])
        {
            // headpos_certainty_msgs.vectors[model].certainty = 0.0;
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
                        CLMTracker::DetectLandmarksInVideo(grayscale_image, depth_image, face_detections[detection_ind],
                         clm_models[model], clm_parameters[model]);
                    }
                    else
                    {
                        ROS_FATAL_STREAM("Standalone images cannot be used in this release");
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
                CLMTracker::DetectLandmarksInVideo(grayscale_image, depth_image, clm_models[model], clm_parameters[model]);
            }
            else
            {
                ROS_FATAL_STREAM("Standalone images cannot be used in this release");
            }

            // e: don't think we're using hog
            double confidence = 0.5 * (1 - clm_model.detection_certainty);
        }

    });
    // used to check if a face is detected in this iteration
    int faceDetected = 0;
    double detection_certainty = 0.0;

    // Go through every model and visualise the results
    for(size_t model = 0; model < clm_models.size(); ++model)
    {
        // Draw the facial landmarks on the face and the bounding box around it
        // if tracking is successful and initialized
        detection_certainty = clm_models[model].detection_certainty;
        double visualisation_boundary = -0.1;
        // Only draw if the reliability is reasonable, the value is slightly ad-hoc
        if(detection_certainty < visualisation_boundary)
        {
            faceDetected = 1;


            retrieveFaceImage(disp_image, clm_models[model]);

            CLMTracker::Draw(disp_image, clm_models[model]);
            if(detection_certainty > 1)     detection_certainty =  1;
            if(detection_certainty < -1)    detection_certainty = -1;
            detection_certainty = (detection_certainty + 1)/(visualisation_boundary +1);

            // displaying tracking certainty
            char certainty_C[255];
            sprintf(certainty_C, "%f", 1-detection_certainty);
            string certainty_st("Certainty: ");
            certainty_st += certainty_C;
            cv::Point certainty_pos;
            certainty_pos = cv::Point(10, 100); // default position on the left top corner
            cv::putText(disp_image, certainty_st, certainty_pos, CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0));

            if (is_assessing) {
                if (!is_child_assessment_done) {
                    mom_child_assessment_tracking_file << 1 << endl;
                    dad_child_assessment_tracking_file << 1 << endl;
                } else if (!is_mom_assessment_done) {
                    mom_assessment_tracking_file << 1 << endl;
                } else {
                    dad_assessment_tracking_file << 1 << endl;
                }
                assessment_tracking_total_num++;
                assessment_tracking_correct_num++;
            }
        } else {
            if (is_assessing) {
                if (!is_child_assessment_done) {
                    mom_child_assessment_tracking_file << 0 << endl;
                    dad_child_assessment_tracking_file << 0 << endl;
                } else if (!is_mom_assessment_done) {
                    mom_assessment_tracking_file << 0 << endl;
                } else {
                    dad_assessment_tracking_file << 0 << endl;
                }
                assessment_tracking_total_num++;
            }
        }

        if (is_assessing) {
            double assessing_time = double(std::clock() - start_assessment_time) / CLOCKS_PER_SEC;
            if (assessing_time > assessment_length * 60) {
                is_assessing = false;
                if (!is_child_assessment_done) {
                    is_child_assessment_done = true;
                    mom_child_assessment_tracking_file << "accuracy" << endl;
                    mom_child_assessment_label_file << "accuracy" << endl;
                    dad_child_assessment_tracking_file << "accuracy" << endl;
                    dad_child_assessment_label_file << "accuracy" << endl;

                    if (assessment_tracking_total_num == 0) {
                        mom_child_assessment_tracking_file << 0 << endl;
                        dad_child_assessment_tracking_file << 0 << endl;
                    } else {
                        mom_child_assessment_tracking_file << (double)assessment_tracking_correct_num / (double)assessment_tracking_total_num << endl;
                        dad_child_assessment_tracking_file << (double)assessment_tracking_correct_num / (double)assessment_tracking_total_num << endl;
                    }
                    mom_child_assessment_tracking_file.close();
                    dad_child_assessment_tracking_file.close();

                    if (assessment_label_total_num == 0) {
                        mom_child_assessment_label_file << 0 << endl;
                        dad_child_assessment_label_file << 0 << endl;
                    } else {
                        mom_child_assessment_label_file << (double)assessment_label_correct_num / (double)assessment_label_total_num << endl;
                        dad_child_assessment_label_file << (double)assessment2_label_correct_num / (double)assessment_label_total_num << endl;
                    }
                    mom_child_assessment_label_file.close();
                    dad_child_assessment_label_file.close();
                } else if (!is_mom_assessment_done) {
                    is_mom_assessment_done = true;
                    mom_assessment_tracking_file << "accuracy" << endl;
                    mom_assessment_label_file << "accuracy" << endl;

                    if (assessment_tracking_total_num == 0) {
                        mom_assessment_tracking_file << 0 << endl;
                    } else {
                        mom_assessment_tracking_file << (double)assessment_tracking_correct_num / (double)assessment_tracking_total_num << endl;
                    }
                    mom_assessment_tracking_file.close();

                    if (assessment_label_total_num == 0) {
                        mom_assessment_label_file << 0 << endl;
                    } else {
                        mom_assessment_label_file << (double)assessment_label_correct_num / (double)assessment_label_total_num << endl;
                    }
                    mom_assessment_label_file.close();
                } else {
                    is_dad_assessment_done = true;
                    dad_assessment_tracking_file << "accuracy" << endl;
                    dad_assessment_label_file << "accuracy" << endl;

                    if (assessment_tracking_total_num == 0) {
                        dad_assessment_tracking_file << 0 << endl;
                    } else {
                        dad_assessment_tracking_file << (double)assessment_tracking_correct_num / (double)assessment_tracking_total_num << endl;
                    }
                    dad_assessment_tracking_file.close();

                    if (assessment_label_total_num == 0) {
                        dad_assessment_label_file << 0 << endl;
                    } else {
                        dad_assessment_label_file << (double)assessment_label_correct_num / (double)assessment_label_total_num << endl;
                    }
                    dad_assessment_label_file.close();

                }
            }
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

    cv::putText(disp_image, fpsSt, cv::Point(10,20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0));

    int num_active_models = 0;

    for( size_t active_model = 0; active_model < active_models.size(); active_model++)
    {
        if(active_models[active_model])
        {
            num_active_models++;
        }
    }

    string training_text = "";
    if (is_training()) {
        string target = "";
        if (train_stage <= num_stages) {
            target = "MOM";
        } else if (train_stage <= 2 * num_stages) {
            target = "DAD";
        } else {
            target = "CHILD";
        }
        training_text = target + " training at stage " + to_string(train_stage) + " (" + get_stage_task(train_stage) + "): " + to_string(num_train_samples);
    } else if (!is_model_trained) {
        if (is_training_done()) { // finish training
            training_text = "Click on the image to start training the model";
        } else {
            string target = "";
            if (train_stage <= num_stages) {
                target = "MOM";
            } else if (train_stage <= 2 * num_stages) {
                target = "DAD";
            } else {
                target = "CHILD";
            }
            training_text = "Click on the image to start training for " + target + " at stage " + to_string(train_stage) + ": " + get_stage_task(train_stage);
        }
    }

    cv::putText(disp_image, training_text, cv::Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255,0,0));

    if (faceDetected)
    {
        num_detected_frames++;
    }
    
    publishImage(disp_image);


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
}

string InitializerHelper::get_stage_task(int stage)
{
    return tasks.at(stage % num_stages);
    /*
    if (0 == stage % num_stages) {
        task = "STRAIGHT FORWARD";
    } else if (1 == stage % num_stages) {
        task = "TURN LEFT";
    } else if (2 == stage % num_stages) {
        task = "TURN RIGHT";
    } else if (3 == stage % num_stages) {
        task = "TURN UP";
    } else if (4 == stage % num_stages) {
        task = "TURN DOWN";
    } else if (5 == stage % num_stages) {
        task = "LEAN LEFT";
    } else if (6 == stage % num_stages) {
        task = "LEAN RIGHT";
    } else if (7 == stage % num_stages) {
        task = "TOP LEFT CORNER OF SCREEN";
    } else if (8 == stage % num_stages) {
        task = "TOP RIGHT CORNER OF SCREEN";
    } else if (9 == stage % num_stages) {
        task = "BOTTOM LEFT CORNER OF SCREEN";
    } else if (10 == stage % num_stages) {
        task = "BOTTOM RIGHT CORNER OF SCREEN";
    } else if (11 == stage % num_stages) {
        task = "CENTER OF SCREEN";
    } else if (12 == stage % num_stages) {
        task = "ROBOT";
    } else if (13 == stage % num_stages) {
        task = "PARENT/CHILD";
    }
    return task;
    */
}

void InitializerHelper::retrieveFaceImage(cv::Mat img, const CLMTracker::CLM& clm_model)
{
    int idx = clm_model.patch_experts.GetViewIdx(clm_model.params_global, 0);
    int n = clm_model.detected_landmarks.rows/2;

    if (clm_model.detected_landmarks.rows > 0 && clm_model.detected_landmarks.cols > 0) {
        int x_min = (int)clm_model.detected_landmarks.at<double>(0);
        int x_max = (int)clm_model.detected_landmarks.at<double>(0);
        int y_min = (int)clm_model.detected_landmarks.at<double>(n);
        int y_max = (int)clm_model.detected_landmarks.at<double>(n);

        for( int i = 0; i < n; ++i)
        {       
            if(clm_model.patch_experts.visibilities[0][idx].at<int>(i))
            {
                int x = (int)clm_model.detected_landmarks.at<double>(i);
                int y = (int)clm_model.detected_landmarks.at<double>(i + n);
                
                x_min = (x_min > x) ? x : x_min;
                x_max = (x_max < x) ? x : x_max;
                y_min = (y_min > y) ? y : y_min;
                y_max = (y_max < y) ? y : y_max;        
            }
        }

        cv::Rect face_contour;
        int width = x_max - x_min;
        int height = y_max - y_min;
        face_contour.x = x_min;
        if (width < height) {
            face_contour.x -= (int)(height - width) / 2;
        }
        face_contour.y = y_min;
        if (height < width) {
            face_contour.y -= (int)(width - height) / 2;
        }
        face_contour.width = (width > height) ? width : height;
        face_contour.height = (width > height) ? width : height;

        if (face_contour.x >= 0 && face_contour.y >= 0 && face_contour.width >= 100 && face_contour.height >= 100 && face_contour.x + face_contour.width <= img.cols && face_contour.y + face_contour.height <= img.rows) {
            cv::Mat face = img(face_contour);
            cv::Size size(100, 100);
            cv::Mat rescaled_face;
            cv::resize(face, rescaled_face, size);
            cv::cvtColor(rescaled_face, rescaled_face, CV_RGB2GRAY); // the method needs grey scale
            cv::imshow("face", rescaled_face);
            cv::waitKey(1);

            if (is_training()) {
                string image_location = face_recognizer_file_location + "train_images/" + to_string(train_stage + 1) + "/" + to_string(num_train_samples) + ".jpg";
                if (!exists(face_recognizer_file_location + "train_images")) {
                    create_directories(face_recognizer_file_location + "train_images");
                }
                if (!exists(face_recognizer_file_location + "train_images/" + to_string(train_stage + 1))) {
                    create_directories(face_recognizer_file_location + "train_images/" + to_string(train_stage + 1));
                }
                imwrite(image_location, rescaled_face);
                num_train_samples++;
            } else if (is_model_trained) {
                int predicted_label = 0;
                double predicted_confidence = 0;
                face_recognizer->predict(rescaled_face, predicted_label, predicted_confidence);

                string role;
                if (predicted_label <= num_stages) {
                    role = "MOM";
                } else if (predicted_label <= 2* num_stages) {
                    role = "DAD";
                } else {
                    role = "CHILD";
                }

                string result = "";

                if (!is_assessing && is_assessment && !is_assessment_done()) {
                    result += "Click on the image to start assessment for ";
                    if (!is_child_assessment_done) {
                        result += "CHILD";
                    } else if (!is_mom_assessment_done){
                        result += "MOM";
                    } else {
                        result += "DAD";
                    }
                } else {
                    if (is_assessing) {
                        face_recognizer_dad->predict(rescaled_face, predicted_label, predicted_confidence);
                        string dad_model_role;
                        if (predicted_label <= num_stages) {
                            dad_model_role = "MOM";
                        } else if (predicted_label <= 2* num_stages) {
                            dad_model_role = "DAD";
                        } else {
                             dad_model_role = "CHILD";
                        }

                        result += "[ASSESSING ";
                        if (!is_child_assessment_done) {
                            mom_child_assessment_label_file << role << endl;
                            dad_child_assessment_label_file << dad_model_role << endl;
                            result += "CHILD";
                        } else if (!is_mom_assessment_done) {
                            mom_assessment_label_file << role << endl;
                            result += "MOM";
                        } else {
                            dad_assessment_label_file << role << endl;
                            result += "DAD";
                        }
                        result += "] ";

                        if (role == assessment_label_correct_answer) {
                            assessment_label_correct_num++;
                        }
                        if (dad_model_role == assessment_label_correct_answer) {
                            assessment2_label_correct_num++;
                        }

                        assessment_label_total_num++;
                    }
                    result += "using ";
                    if (is_using_mom_model) {
                        result += "MOM ";
                    } else {
                        result += "DAD ";
                    }
                    result += "model to predict: " + role + " with confidence: " + to_string(predicted_confidence);
                }

                cv::putText(img, result, cv::Point(10,60), CV_FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255,0,0));

            }
        }
    }
}

void InitializerHelper::switch_role()
{
    if (is_using_mom_model) {
        face_recognizer = face_recognizer_dad;
        is_using_mom_model = false;
        std::cout << "switch to DAD" << std::endl;
    } else {
        face_recognizer = face_recognizer_mom;
        is_using_mom_model = true;
        std::cout << "switch to MOM" << std::endl;
    }
}

InitializerHelper::InitializerHelper(string _name, string _loc) : 
    executable_location(_loc), 
    imageTransport(nodeHandle),
    is_train(false),
    is_assessing(false),
    is_child_assessment_done(false),
    is_mom_assessment_done(false),
    is_dad_assessment_done(false),
    is_model_trained(false),
    train_stage(0),
    num_train_samples(0),
    face_recognizer_mom(cv::face::createEigenFaceRecognizer()),
    face_recognizer_dad(cv::face::createEigenFaceRecognizer()),
    assessment_tracking_total_num(0),
    assessment_tracking_correct_num(0),
    assessment_label_correct_answer(""),
    assessment_label_total_num(0),
    assessment_label_correct_num(0),
    assessment2_label_correct_num(0),
    is_using_mom_model(false)
{
    ROS_INFO("Called constructor...");

    string _cam = "/usb_cam";
    nodeHandle.getParam("cam", _cam);
    nodeHandle.getParam("face_recognizer_file_location", face_recognizer_file_location);
    nodeHandle.getParam("single_face_recognizer_assessment_file_location", face_assessment_file_location);

    nodeHandle.getParam("is_get_data", is_get_data);
    nodeHandle.getParam("is_train", is_to_train);
    nodeHandle.getParam("is_assessment", is_assessment);
    nodeHandle.getParam("assessment_length", assessment_length);

    if (is_get_data) {
        if (exists(face_recognizer_file_location + "train_images")) {
            remove_all(face_recognizer_file_location + "train_images");
        }
        if (exists(face_recognizer_file_location + "face_recognizer_model_mom.xml")) {
            remove(face_recognizer_file_location + "face_recognizer_model_mom.xml");
        }
        if (exists(face_recognizer_file_location + "face_recognizer_model_dad.xml")) {
            remove(face_recognizer_file_location + "face_recognizer_model_dad.xml");
        }
    } else {
        is_train = false;
        train_stage = num_stages * num_role;
        is_model_trained = !is_to_train;
        if (is_model_trained) {
            face_recognizer_mom = cv::face::createEigenFaceRecognizer();
            face_recognizer_mom->load(face_recognizer_file_location + "face_recognizer_model_mom.xml");
            face_recognizer_dad = cv::face::createEigenFaceRecognizer();
            face_recognizer_dad->load(face_recognizer_file_location + "face_recognizer_model_dad.xml");
        }
    }

    if (is_assessment) {
        if (exists(face_assessment_file_location)) {
            remove_all(face_assessment_file_location);
        }
        if (exists(face_assessment_file_location + "mom/")) {
            remove_all(face_assessment_file_location + "mom/");
        }
        if (exists(face_assessment_file_location + "dad/")) {
            remove_all(face_assessment_file_location + "dad/");
        }
        create_directories(face_assessment_file_location);
        create_directories(face_assessment_file_location + "mom/");
        create_directories(face_assessment_file_location + "dad/");

        mom_child_assessment_label_file.open(face_assessment_file_location + "mom/" + "child_label.txt", std::ios_base::app);
        mom_child_assessment_tracking_file.open(face_assessment_file_location + "mom/" + "child_tracking.txt", std::ios_base::app);
        mom_assessment_label_file.open(face_assessment_file_location + "mom/" + "parent_label.txt", std::ios_base::app);
        mom_assessment_tracking_file.open(face_assessment_file_location + "mom/" + "parent_tracking.txt", std::ios_base::app);

        dad_child_assessment_label_file.open(face_assessment_file_location + "dad/" + "child_label.txt", std::ios_base::app);
        dad_child_assessment_tracking_file.open(face_assessment_file_location + "dad/" + "child_tracking.txt", std::ios_base::app);
        dad_assessment_label_file.open(face_assessment_file_location + "dad/" + "parent_label.txt", std::ios_base::app);
        dad_assessment_tracking_file.open(face_assessment_file_location + "dad/" + "parent_tracking.txt", std::ios_base::app);
    }

    switch_role(); // initially is mom

    // raw camera image
    imageSubscriber = imageTransport.subscribe(_cam+"/image_raw",1,&InitializerHelper::callback, this);

    namedWindow("Face Recognition Training");
    setMouseCallback("Face Recognition Training", mouse_callback, this);

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

    clm_parameters.push_back(clm_params);

    // Get the input output file parameters

    // ERICNOTE: the following section deals with image files
    // Indicates that rotation should be with respect to camera plane or with respect to camera
    video_input = true;
    images_as_video = false;

    vector<vector<string> > input_image_files;

    int num_faces_max = 1;
    CLMTracker::CLM clm_model2(clm_parameters[0].model_location);
    clm_model2.face_detector_HAAR.load(clm_parameters[0].face_detector_location);
    clm_model2.face_detector_location = clm_parameters[0].face_detector_location;

    clm_models.reserve(num_faces_max);

    clm_models.push_back(clm_model2);
    active_models.push_back(false);

    total_frames = -1;
    reported_completion = 0;

    frame_count = 0;
    num_detected_frames = 0;

};
