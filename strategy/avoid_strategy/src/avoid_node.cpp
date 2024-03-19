#include "../src/avoid.h"

using namespace unit2;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"unitwork2");
    ros::NodeHandle nh;
    ros::start();
    unit2::Car car;
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        car.process();
        ros::spinOnce();
        loop_rate.sleep();
    }
}