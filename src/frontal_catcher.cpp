#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

static const char WINDOW[] = "Frontal catcher";

class Catcher {
    
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub;
 
    public:
        Catcher() : it(nh) {
            image_sub = it.subscribe("retranslator/cv_front", 1, &Catcher::showImage, this);
            cv::namedWindow(WINDOW);
        }
        
        ~Catcher() {
            cv::destroyWindow(WINDOW);
        }
        
        void showImage(const sensor_msgs::ImageConstPtr & msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                ROS_INFO("%s",cv_ptr->header.frame_id.c_str());
            }
            catch (cv_bridge::Exception & e) {
                ROS_ERROR("%s happend", e.what());
                return;
            }
            
            cv::imshow(WINDOW, cv_ptr->image);
            cv::waitKey(27);
        }
};

int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "frontal_catcher");
   
    Catcher * catcher = new Catcher();

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete catcher;
    return 0;
}
