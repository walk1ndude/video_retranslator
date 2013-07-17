#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/Navdata.h>

class VideoRetranslator {

    public:
        explicit VideoRetranslator();

    private:
        ros::NodeHandle nodeHandle;
        ros::ServiceClient cameraClient;
        ros::Subscriber navdata;

        image_transport::Publisher frontalCamera;
        image_transport::Publisher bottomCamera;

        image_transport::ImageTransport imageTransport;
        image_transport::Subscriber frontalCameraImage;
        image_transport::Subscriber bottomCameraImage;

        cv_bridge::CvImagePtr cvPtr;
       
        int framesToSwitch;
        int framesNotToProcess;

        cv::Mat undistorted;
        cv::Mat cameraMatrix[2];
        cv::Mat distortion[2];

        ardrone_autonomy::CamSelect camSelect;

        void subscribe();
        void publish();
        void setServices();
        void loadCalibData(const char * path);
        
        void getFrontalImage(const sensor_msgs::ImageConstPtr & msg);
        void getBottomImage(const sensor_msgs::ImageConstPtr & msg);
        void getImage(const sensor_msgs::ImageConstPtr & msg, int channel);

        void setCamChannel(int camera);

        void getNavdata(const ardrone_autonomy::Navdata & navdata);

        void removeDistortion(cv::Mat & image, int camera);
};

