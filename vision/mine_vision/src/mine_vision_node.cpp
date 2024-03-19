#include "mineImg.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mine_vision_node");
    
    MineDetector detector;
    ros::Rate loop_rate(30);

    while(ros::ok()) 
    {
        detector.imageProcess();
        detector.publishMine();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}