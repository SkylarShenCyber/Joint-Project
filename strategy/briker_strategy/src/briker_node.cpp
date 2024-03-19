#include "briker.h"

using namespace unit;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"unit_work");
    ros::NodeHandle nh;
    ros::start();
    unit::car Car;
    ros::Rate loop_rate(30);
    //Car.position_sub_=nh.subscribe<std_msgs::StringConstPtr>("pointParam",1000,&car::pointCallback);
    //Car.distance_sub_=nh.subscribe<std_msgs::StringConstPtr>("distanceParam",1000,&car::distanceCallback);
    while(ros::ok())
    {
        Car.process();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

