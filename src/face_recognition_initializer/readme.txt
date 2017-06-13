To run clm_ros_wrapper, the face recognizer model should be trained.


************************************1. train the model******************************************************

To train the model from the very beginning:
roslaunch face_recognition_initializer.launch
Then follow the instruction

To view the prediction with the current model:
roslaunch face_recognition_predict.launch

To train with the current images and view the prediction:
face_recognition_train_predict.launch
Then follow the instruction

To change the training method (the angle to take of each role):
1) add the method to / delete the method in
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
at the top of initializerHelper.cpp
2) change the value of
int static const num_stages = 14;
in CLMWrapper.cpp

All of the above take only one face, and does not support multiple faces.

To create a new launch file, here is the template:

<launch>

    <!-- camera frame to world frame transformation matrix row-major representation-->
    <!-- <rosparam param = "transformation_cf2intermediate_frame">[-1, 0, 0, 280, 0, 0, -1, 410, 0, -1, 0, 340, 0, 0 , 0, 1]</rosparam>
    <rosparam param = "transformation_intermediate_frame2wf">[0.9130, -0.4078, 0, 0, 0.4078, 0.9130, 0, 0, 0, 0, 1, 0, 0,0 , 0, 1]</rosparam> -->
    
    <rosparam param = "transformation_intermediate_frame2wf">[-1,0,0,0.0,0,0,-1,0.0,0,-1,0,0.0,0,0,0,1]</rosparam>

    <rosparam param = "transformation_cf2intermediate_frame">[1,0,0,400, 0,1,0, 450, 0,0,1, 410,0,0,0,1]</rosparam>

    <!-- world frame to robot frame transformation matrix row-major representation-->
    <!-- <rosparam param = "transformation_wf2rf">[-0.7041, 0.7100, 0, 340, -0.7100, -0.7041, 0, -100, 0, 0, 1, 0, 0,0 , 0, 1]</rosparam> -->
    
    <rosparam param = "transformation_wf2rf">[-0.23313, -0.97244, 0, 315.57679, 0.97244, -0.23313, 0, -586.38482, 0, 0, 1, 0, 0 , 0, 0, 1]</rosparam>

    <!-- screen parameters - dimensions and angles -->
    <param name="screenAngleInDegrees" type="int" value="46"/>
    <param name="screenHeight" type="double" value="380"/> <!-- mm -->
    <param name="screenWidth" type="double" value="520"/><!-- mm -->

    <!-- calibration -->
    <param name="offset_hfv_wf_z" type="double" value="0"/> <!-- "-0.1"-->
    <param name="offset_head_position_cf_z" type="double" value="200"/> <!-- box size -->

    <!-- robot parameters - radius for inside/outside check-->
    <param name="robot_radius" type="double" value="220"/> <!-- mm -->

    <param name="ns" type="string" value="clm_ros_wrapper_2"/>
    <param name="cam" type="string" value="usb_cam_2"/>

    <!-- face_recognizer_file_location: the location of where the images and the model is stored. -->
    <param name="face_recognizer_file_location" type="string" value="/home/sar/face_analyzer_data/"/>

    <!-- single_face_recognizer_assessment_file_location: the file location where the assessment results are saved.>
    <param name="single_face_recognizer_assessment_file_location" type="string" value="/home/sar/face_analyzer_assessment/single_face.txt"/>

    <!-- is_get_data: whether to get new images for training. All existing images and model will be deleted -->
    <param name="is_get_data" type="boolean" value="true"/>

    <!-- is_train: whether to train a new model with the existing images. There must be images available if the model needs to be trained. If is_get_data is true, then regardless what the value for is_train, a new model will be trained. -->
    <param name="is_train" type="boolean" value="false"/>

    <!-- is_assessment: whether to do an assessment in the end. This one does only the single face assessment -->
    <param name="is_assessment" type="boolean" value="true">

    <!-- assessment_length: how long the assessment is. The unit is in minutese. -->
    <param name="assessment_length" type="double" value="3"/>

    <!-- the value should be the same as the setting as in the clm_ros_wrapper -->
    <node name="usb_cam_2" pkg="usb_cam" type="usb_cam_node" output="screen" >
        <param name="video_device" value="/dev/video1" />
        <param name="image_width" value="1280" /> <!-- 1280 -->
        <param name="image_height" value="720" /> <!-- 720-->
        <param name="pixel_format" value="yuyv" />
        <param name="camera_frame_id" value="usb_cam" />
        <param name="io_method" value="mmap"/>
    </node>

    <node pkg="clm_ros_wrapper" type="face_recognition_initializer" name="face_recognition_initializer" output="screen" />

</launch>



************************************2. use the model******************************************************

To use clm_ros_wrapper, the model should already be trained. Otherwise there will be an error. Some parameter in the launch file that is related to this functionality:

<!-- face recognition parameters-->
<param name="face_recognizer_file_location" type="string" value="/home/sar/face_analyzer_data/"/>
<param name="child_confidence_threshold" type="int" value="3000"/>
<param name="parent_confidence_threshold" type="int" value="3000"/>

face_recognizer_file_location: the location of where the face recognizer model is stored. This must be the same as the face_recognizer_file_location above.

child_confidence_threshold: the threshold of the matching distance of the child. A higher matching distance means the current face is more distant from the model. So a value higher than this threshold will be recognize as not the child.

parent_confidence_threshold: the threshold that is similar as the child_confidence_threshold. But this is the threshold for parent.

At the top of CLMWrapper.cpp:
The label of child, parent and other is defined as:
int static const child = 1;
int static const parent = 2;
int static const other = -1; (this is used in user_tracking_fusion.py in sar_core)
child and parent is defined in the same way as the ones in the message.
other, in the message, is defined as 3, rather than -1.

***********************************3. Assess Both faces**************************************************

The file should be launched from sar core. The flag in clm launch file is:

        <param name="is_assessment" type="bool" value="false" />
        <param name="assessment_length" type="double" value="0.25" />

is_assessment is whether this is an assessment.
assessment_length is the length of assessment, which is in minutes.

The modified version is called face_recognition_two_faces_assessment.launch
