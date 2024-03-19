#ifndef UNIT2_AVOID_H
#define UNIT2_AVOID_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>
#include <ros/package.h>
#include "strategy.h"
#include <sstream>
#include "avoid_strategy/WalkingParam.h"
#include "mine_vision/Rect.h"
#include <math.h>

#define degree2radian (M_PI/180.0f)
#define radian2degree (180.0f/M_PI)

namespace unit2
{
    class Car : public Strategy
    {
        public:
            enum State
            {
                START,
                JUDGE,
                WALK,
                ADJUST,
                SAFETY
            };

            Car();
            ~Car();

            void process();
            boost::thread point_thread;
            void pointThread();
            void pointCallback(const mine_vision::RectConstPtr& msg);
            boost::thread distance_thread;
            void distanceThread();
            void distanceCallback(const std_msgs::StringConstPtr& msg);

            void startMode();
            void judge();
            void walk();
            void adjust();
            void safety();
            void setWalkingParam(int x_move,int y_move,int angle,int x_speed,int y_speed,int angle_speed);

            ros::Subscriber position_sub_;
            ros::Subscriber distance_sub_;
            ros::Publisher walk_pub_;
            ros::Publisher command_pub_;
            ros::Publisher command_pub_2;//run start
            ros::Publisher command_pub_3;//arm start
            ros::Publisher x_move_pub_;
            ros::Publisher y_move_pub_;
            ros::Publisher angle_pub_;
            ros::Publisher x_speed_pub_;
            ros::Publisher y_speed_pub_;
            ros::Publisher angle_speed_pub_;

            avoid_strategy::WalkingParam next_move;
            std_msgs::Int16 command;
            std_msgs::Int16 command2;
            
            int process_mode,premode;
            int x1,y1,x2,y2,dis,x_middle;
            int SPIN_RATE;
            int safety_flag;
            float degree;
            float length;

            int fabs(int a);

    };
}
#endif