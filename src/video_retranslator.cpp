#include "video_retranslator.h"

#include <sensor_msgs/image_encodings.h>

#define AUTONOMY_FRONTAL_CAMERA_TOPIC "ardrone/front/image_raw"
#define AUTONOMY_BOTTOM_CAMERA_TOPIC "ardrone/bottom/image_raw"

#define NAVDATA_TOPIC "ardrone/navdata"

#define CAMERA_CHANNEL_SERVICE "ardrone/setcamchannel"

#define CV_FRONT_IMAGE "retranslator/cv_front"
#define CV_BOTTOM_IMAGE "retranslator/cv_bottom"

#define FRONTAL_CAMERA 0
#define BOTTOM_CAMERA 1

#define FRAME_SWITCH_MAX 10
#define FRAMES_NOT_TO_PROCESS 4

VideoRetranslator::VideoRetranslator() : imageTransport(nodeHandle) {
    subscribe();
    publish();
    setServices();
    loadCalibData("cameraParams.yml");
    setCamChannel(FRONTAL_CAMERA); 
}

void VideoRetranslator::subscribe() {
   frontalCameraImage = imageTransport.subscribe(AUTONOMY_FRONTAL_CAMERA_TOPIC, 1, &VideoRetranslator::getFrontalImage, this);
   bottomCameraImage = imageTransport.subscribe(AUTONOMY_BOTTOM_CAMERA_TOPIC, 1, &VideoRetranslator::getBottomImage, this);
   navdata = nodeHandle.subscribe(NAVDATA_TOPIC, 1, &VideoRetranslator::getNavdata, this);   
}

void VideoRetranslator::publish() {
    frontalCamera = imageTransport.advertise(CV_FRONT_IMAGE, 1);
    bottomCamera = imageTransport.advertise(CV_BOTTOM_IMAGE, 1);
}

void VideoRetranslator::setServices() {
    cameraClient = nodeHandle.serviceClient<ardrone_autonomy::CamSelect>(CAMERA_CHANNEL_SERVICE);
}

void VideoRetranslator::loadCalibData(const char * path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    
    fs["cameraMatrixFront"] >> cameraMatrix[FRONTAL_CAMERA];
    fs["distCoeffsFront"] >> distortion[FRONTAL_CAMERA];

    fs["cameraMatrixBottom"] >> cameraMatrix[BOTTOM_CAMERA];
    fs["distCoeffsBottom"] >> distortion[BOTTOM_CAMERA];

    assert(!cameraMatrix[FRONTAL_CAMERA].empty() && !distortion[FRONTAL_CAMERA].empty() &&
            !cameraMatrix[BOTTOM_CAMERA].empty() && !distortion[BOTTOM_CAMERA].empty());

    fs.release();
}

void VideoRetranslator::getImage(const sensor_msgs::ImageConstPtr & msg, int currentChannel) {
   
    if (framesNotToProcess == 0) {
    
        try {
            
            ROS_INFO("PROCESSING IMAGE");
            
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            removeDistortion(cvPtr->image, currentChannel);
            cvPtr->image = undistorted;

            if (currentChannel == FRONTAL_CAMERA) {
                frontalCamera.publish(cvPtr->toImageMsg());
            }
            else {
                bottomCamera.publish(cvPtr->toImageMsg());
            }
            
            ROS_INFO("IMAGE SEND");

            if ( ++ framesToSwitch == FRAME_SWITCH_MAX ) {
                setCamChannel(currentChannel == FRONTAL_CAMERA ? BOTTOM_CAMERA : FRONTAL_CAMERA);
            }
        }
        catch (cv_bridge::Exception & e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    else {
        framesNotToProcess -- ;
    }
}

void VideoRetranslator::getFrontalImage(const sensor_msgs::ImageConstPtr & msg) {
    getImage(msg, FRONTAL_CAMERA);
}

void VideoRetranslator::getBottomImage(const sensor_msgs::ImageConstPtr & msg) {
    getImage(msg, BOTTOM_CAMERA);
}

void VideoRetranslator::getNavdata(const ardrone_autonomy::Navdata & navdata) {
    ROS_INFO("rotX: %f",navdata.rotX);
}

void VideoRetranslator::setCamChannel(int camera) {
    framesToSwitch = 0;
    framesNotToProcess = FRAMES_NOT_TO_PROCESS;
    
    camSelect.request.channel = camera;
    cameraClient.call(camSelect);
}

void VideoRetranslator::removeDistortion(cv::Mat & image, int camera) {
    cv::undistort(image, undistorted, cameraMatrix[camera], distortion[camera]);
}
