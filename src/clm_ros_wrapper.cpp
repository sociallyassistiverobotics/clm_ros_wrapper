///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, University of Cambridge,
// all rights reserved.
//
// THIS SOFTWARE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY. OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Notwithstanding the license granted herein, Licensee acknowledges that certain components
// of the Software may be covered by so-called “open source” software licenses (“Open Source
// Components”), which means any software licenses approved as open source licenses by the
// Open Source Initiative or any substantially similar licenses, including without limitation any
// license that, as a condition of distribution of the software licensed under such license,
// requires that the distributor make the software available in source code format. Licensor shall
// provide a list of Open Source Components for a particular version of the Software upon
// Licensee’s request. Licensee will comply with the applicable terms of such licenses and to
// the extent required by the licenses covering Open Source Components, the terms of such
// licenses will apply in lieu of the terms of this Agreement. To the extent the terms of the
// licenses applicable to Open Source Components prohibit any of the restrictions in this
// License Agreement with respect to such Open Source Component, such restrictions will not
// apply to such Open Source Component. To the extent the terms of the licenses applicable to
// Open Source Components require Licensor to make an offer to provide source code or
// related information in connection with the Software, such offer is hereby made. Any request
// for source code or related information should be directed to cl-face-tracker-distribution@lists.cam.ac.uk
// Licensee acknowledges receipt of notices for the Open Source Components for the initial
// delivery of the Software.

//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite one of the following works (the related one preferrably):
//
//       Tadas Baltrusaitis, Peter Robinson, and Louis-Philippe Morency. 3D
//       Constrained Local Model for Rigid and Non-Rigid Facial Tracking.
//       IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2012.    
//
//       Tadas Baltrusaitis, Peter Robinson, and Louis-Philippe Morency. 
//       Constrained Local Neural Fields for robust facial landmark detection in the wild.
//       in IEEE Int. Conference on Computer Vision Workshops, 300 Faces in-the-Wild Challenge, 2013.    
//
//       Tadas Baltrusaitis, Marwa Mahmoud, and Peter Robinson.
//		 Cross-dataset learning and person-specific normalisation for automatic Action Unit detection
//       Facial Expression Recognition and Analysis Challenge 2015,
//       IEEE International Conference on Automatic Face and Gesture Recognition, 2015
//
//       Erroll Wood, Tadas Baltrušaitis, Xucong Zhang, Yusuke Sugano, Peter Robinson, and Andreas Bulling
//		 Rendering of Eyes for Eye-Shape Registration and Gaze Estimation
//       in IEEE International. Conference on Computer Vision (ICCV), 2015
//
///////////////////////////////////////////////////////////////////////////////


#include "CLM_core.h"

#include <fstream>
#include <sstream>

#include <opencv2/videoio/videoio.hpp>  // Video write
#include <opencv2/videoio/videoio_c.h>  // Video write
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
#include "std_msgs/String.h"

#include <clm_ros_wrapper/ClmHeads.h>
#include <clm_ros_wrapper/ClmEyeGaze.h>
#include <clm_ros_wrapper/ClmFacialActionUnit.h>

#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

using namespace std;
using namespace cv;

using namespace boost::filesystem;

class ClmWrapper
{
private:
    std::string name;

    ros::NodeHandle nodeHandle;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber     imageSubscriber;

    ros::Publisher headsPublisher;

    cv::Mat img_in;

    /**
     * Callback on the subscriber's topic.
     * @param msgIn an RGB image
     */
    void callback(const sensor_msgs::ImageConstPtr& msgIn)
    {
        ROS_INFO("Triggered callback!");

        // Let's convert the ROS image to OpenCV image format
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msgIn, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        img_in = cv_ptr->image.clone();
        // cv::Mat img_out = img_in.clone();

        // std_msgs::String msg;
        // std::stringstream ss;
        // ss << "hello world";
        // msg.data = ss.str();

        // // just publish a hello world for now
        // clm_heads_pub.publish( msg );
    }

public:

    ClmWrapper(string _name) : name(_name), imageTransport(nodeHandle)
    {
        ROS_INFO("Constructor");

        imageSubscriber = imageTransport.subscribe("/usb_cam/image_raw",1,&ClmWrapper::callback, this);
        headsPublisher  = nodeHandle.advertise<clm_ros_wrapper::ClmHeads>("/clm_ros_wrapper/heads",1);
    };

    ~ClmWrapper() {};
};

int main (int argc, char **argv)
{
	ROS_INFO("Starting..");

    std::string name = "clm_ros_wrapper";

    ros::init(argc, argv, name.c_str());

    ClmWrapper clm_wrapper(name);
    ros::spin();

  return 0;
}

