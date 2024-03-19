#include "lineStrategy.h"

using namespace line;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_strategy_node");
    
    lineStrategy strategy;
   
    ros::Rate loop_rate(30);


    while (ros::ok())
    {
        strategy.process();
        strategy.testINFO();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    
    return 0;
}
