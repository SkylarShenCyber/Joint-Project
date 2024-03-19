#include "squareImg.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "square_vision_node");
    
    SquareDetector detector;
    ros::Rate loop_rate(30);

    while(ros::ok()) 
    {
        detector.imageProcess();
        detector.publishInfo();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}