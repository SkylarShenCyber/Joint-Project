#include "avoid.h"

namespace unit2
{
    Car::Car()
    :   SPIN_RATE(30)
    {
        ros::NodeHandle nh(ros::this_node::getName());
        boost::thread point_thread=boost::thread(boost::bind(&Car::pointThread,this));
        // walk_pub_=nh.advertise<avoid_strategy::WalkingParam>("walkPAram",1000);
        command_pub_=nh.advertise<std_msgs::Int16>("command",1);
        command_pub_2=nh.advertise<std_msgs::Int16>("command2",1);
        x_move_pub_=nh.advertise<std_msgs::Int16>("walkParam_x_move",1000);
        y_move_pub_=nh.advertise<std_msgs::Int16>("walkParam_y_move",1000);
        angle_pub_=nh.advertise<std_msgs::Int16>("walkParam_angle",1000);
        x_speed_pub_=nh.advertise<std_msgs::Int16>("walkParam_x_speed",1000);
        y_speed_pub_=nh.advertise<std_msgs::Int16>("walkParam_y_speed",1000);
        angle_speed_pub_=nh.advertise<std_msgs::Int16>("walkParam_angle_speed",1000);
        startMode();
        boost::thread distance_thread=boost::thread(boost::bind(&Car::distanceThread,this));
    }

    Car::~Car(){}

    void Car::process()
    {
        if(process_mode==SAFETY)
            safety();
        else if(process_mode==ADJUST)
            adjust();
        else if(process_mode==WALK)
            walk();
        else if(process_mode==JUDGE)
            judge();
    }

    void Car::pointCallback(const mine_vision::RectConstPtr& msg)
    {
        x1=msg->upleftPoint.x;
        y1=msg->upleftPoint.y;
        x2=msg->downrightPoint.x;
        y2=msg->downrightPoint.y;
        ROS_INFO("upleft: x:%d y:%d\n",x1,y1);
        ROS_INFO("downright: x:%d y:%d\n",x2,y2);
        // ROS_INFO("%s",msg->data.c_str());
        // std::stringstream ss;
        // ss<<msg->data.c_str();
        // ss>>x1>>y1>>x2>>y2>>x_middle;
    }

    void Car::pointThread()
    {
            ROS_INFO("point get");
            ros::NodeHandle nh(ros::this_node::getName());
            position_sub_=nh.subscribe("/mine_vision/target_info",1000,&Car::pointCallback,this);
    }

    void Car::distanceCallback(const std_msgs::StringConstPtr& msg)
    {
        ROS_INFO("%s",msg->data.c_str());
        std::stringstream s;
        s<<msg->data.c_str();
        s>>x1>>y1>>x2>>y2>>x_middle;
    }

    void Car::distanceThread()
    {
        ROS_INFO("distance get");
        ros::NodeHandle nh(ros::this_node::getName());
        distance_sub_=nh.subscribe("distanceParam",1000,&Car::distanceCallback,this); 
    }

    int Car::fabs(int a)
    {
        if(a>=0)
            return a;
        else 
            return -a;
    }

    void Car::startMode()
    {
        ROS_INFO("start!");
        command.data=1;
        command_pub_.publish(command);
        command2.data=2;
        command_pub_2.publish(command2);
        process_mode=JUDGE;
        premode=START;
    }

    void Car::judge()
    {   
        ROS_INFO("judging");
        setWalkingParam(0,0,0,0,0,0);
        usleep(500*1000);
        if(x1!=0&&x2!=0&&y1!=0&&y2!=0)
        {
            premode=JUDGE;
            process_mode=SAFETY;
        }
        else
        {
            premode=JUDGE;
            process_mode=WALK;
        }
    }

    void Car::safety()
    {
        ROS_INFO("safety");
        if(fabs(y1-y2)<25)
            safety_flag=1;
        else
            safety_flag=0;
        
        if(safety_flag)
        {
            premode=SAFETY;
            process_mode=WALK;
        }
        else
        {
            if((fabs(x1-5)<5)||(fabs(x2-195)))// if extremly left or right
            {
                premode=SAFETY;
                process_mode=WALK;
            }
            else
            {
                premode=SAFETY;
                process_mode=ADJUST;
            }
        }
    }

    void Car::walk()
    {
        if(premode==JUDGE)
        {
            ROS_INFO("aimless walking");
            setWalkingParam(0,50,0,0,50,0);
            usleep(500*1000);
            premode=WALK;
            process_mode=JUDGE;
        }
        else if(premode==SAFETY)
        {
            ROS_INFO("safely walking");
            setWalkingParam(0,20,0,0,20,0);
            usleep(500*1000);
            premode=WALK;
            process_mode=JUDGE;
        }
        else if(premode==ADJUST)
        {
            ROS_INFO("avoid walking");
            length=tan(degree*degree2radian);
            setWalkingParam(0,length,0,0,30,0);
            usleep(500*1000);
            premode=WALK;
            process_mode=JUDGE;
        }//need consideration
    }

    void Car::adjust()
    {
        ROS_INFO("turning");
        degree=atan((fabs(x1-x2)/2-fabs(100-x_middle))/dis)*radian2degree;
        setWalkingParam(0,0,degree,0,0,30);
        usleep(500*1000);
        premode=ADJUST;
        process_mode=WALK;
    }

    void Car::setWalkingParam(int x_move,int y_move,int angle,int x_speed,int y_speed,int angle_speed)
    {
        // next_move.x_move=x_move;
        // next_move.y_move=y_move;
        // next_move.angle=angle;
        // next_move.x_speed=x_speed;
        // next_move.y_speed=y_speed;
        // next_move.angle_speed=angle_speed;
        // walk_pub_.publish(next_move);
        std_msgs::Int16 _x_move,_y_move,_angle,_x_speed,_y_speed,_angle_speed;
        _x_move.data=x_move;
        _y_move.data=y_move;
        _angle.data=angle;
        _x_speed.data=x_speed;
        _y_speed.data=y_speed;
        _angle_speed.data=angle_speed;
        x_move_pub_.publish(_x_move);
        y_move_pub_.publish(_y_move);
        angle_pub_.publish(_angle);
        x_speed_pub_.publish(_x_speed);
        y_speed_pub_.publish(_y_speed);
        angle_speed_pub_.publish(_angle_speed);
    }
}