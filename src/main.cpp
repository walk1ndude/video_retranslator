#include "video_retranslator.h"

int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "video_retranslator_node");
   
    VideoRetranslator * retranslator = new VideoRetranslator();
   
    ros::Rate loopRate(30);

    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }

    delete retranslator;

    return 0;

}
