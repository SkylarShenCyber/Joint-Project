#include "lineImg.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_vision_node");
    
    LineDetector detector;
    ros::Rate loop_rate(30);

    while(ros::ok()) 
    {
        detector.imageProcess();
        detector.publishLine();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}