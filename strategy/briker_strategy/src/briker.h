#ifndef UNIT_BRIKER_H_
#define UNIT_BRIKER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include "strategy.h"
#include "briker_strategy/WalkingParam.h"
#include "square_vision/Square.h"
#include "square_vision/Destination.h"

#define _CATCH 1
#define _UP 2
#define _DOWN 3

namespace unit
{
    class car : public Strategy
    {
    public:
        enum State
        {
            SEARCH,
            WALK_FAR,
            WALK_ACCURATE,
            ADJUSTTOWARD,
            LIFTARM,
            DOWNARM,
            MYCATCH,
            STOP,
            TODESTINATION,
            TURN,
        };

        car();
        ~car();
    
    //protected:
        //boost::thread process_thread;
        //void processThread();
        void process();
        boost::thread point_thread;
        void pointThread();
        void pointCallback(const std_msgs::StringConstPtr& msg);
        boost::thread distance_thread;
        void distanceThread();
        void distanceCallback(const std_msgs::StringConstPtr& msg);

        void startMode();
        void search();
        void walkFar();
        void walkAccurate();
        void adjustToward();
        void downArm();
        void liftArm();
        void myCatch();
        void put();
        void turn();
        void stop();
        void setWalkingParam(int x_move,int y_move,int angle,int x_speed,int y_speed,int angle_spped);
        void setArmMode(int mode);
    
        ros::Subscriber position_sub_;
        ros::Subscriber distance_sub_;
        ros::Publisher walk_pub_;
        ros::Publisher arm_pub_;
        ros::Publisher command_pub_;
        ros::Publisher command_pub_2;
        ros::Publisher command_pub_3;
        ros::Publisher command_pub_3_no;
        briker_strategy::WalkingParam next_move;
        std_msgs::Int16 next_mode;
        std_msgs::Int16 command;
        std_msgs::Int16 command2;//run start
        std_msgs::Int16 command3;//arm start
        std_msgs::Int16 command3_no;//arm close
        ros::Publisher x_move_pub_;
        ros::Publisher y_move_pub_;
        ros::Publisher angle_pub_;
        ros::Publisher x_speed_pub_;
        ros::Publisher y_speed_pub_;
        ros::Publisher angle_speed_pub_;
        
        int process_mode,premode;
        int x1,y1,x2,y2,dis;
        int SPIN_RATE;
        
        int fabs(int a);
        
    };
}

#endif
