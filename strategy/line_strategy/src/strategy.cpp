#include "lineStrategy.h"

#include<cmath>

namespace line {

void lineStrategy::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    img_height = msg->height;
    img_width = msg->width;
    //ROS_INFO("img_height: %d\nimg_width: %d",img_height,img_width);
    return;
}

void lineStrategy::leftCallback(const line_vision::Line::ConstPtr& msg)
{
    left.clear();
    Point tmp;
    tmp.x = msg->topPoint.x;
    tmp.y = msg->topPoint.y;
    left.push_back(tmp);//先压入上面的点，后压入下面的点，遍历时上面的点在前，下面的点在后
    tmp.x = msg->bottomPoint.x;
    tmp.y = msg->bottomPoint.y;
    left.push_back(tmp);
    // ROS_INFO("leftCallback running");
    // ROS_INFO("%lf",msg->topPoint.x);
    return;
}

void lineStrategy::rightCallback(const line_vision::Line::ConstPtr& msg)
{
    right.clear();
    Point tmp;
    tmp.x = msg->topPoint.x;
    tmp.y = msg->topPoint.y;
    right.push_back(tmp);//先压入上面的点，后压入下面的点，遍历时上面的点在前，下面的点在后
    tmp.x = msg->bottomPoint.x;
    tmp.y = msg->bottomPoint.y;
    right.push_back(tmp);
    //ROS_INFO("rightCallback running");
    return;
}

void lineStrategy::process()
{

    setWalkingParam(0,0,0,0,0,0);
    if(process_mode==ADJUSTTOWARD)
        adjustToward();
    else if(process_mode==WALK_FAR)
        walkFar();
    else if(process_mode==WALK_ACCURATE)
        walkAccurate();
    else if(process_mode==STOP)
        stop();
    
    return;
}

void lineStrategy::testINFO()
{
    for (size_t i = 0; i < left.size(); i++)
    {
        // ROS_INFO("leftsize:%d",int(left.size()));
        ROS_INFO("left x:%d  y:%d",left[i].x,left[i].y);       
    }
    for (size_t i = 0; i < right.size(); i++)
    {
        ROS_INFO("right x:%d  y:%d",right[i].x,right[i].y);
    }
    // ROS_INFO("test running");
    return;
    
}

lineStrategy::lineStrategy():
    nh_(ros::this_node::getName()),
    it_(this->nh_)
{   
    process_mode=ADJUSTTOWARD;
    premode=ADJUSTTOWARD;
    img_sub_ = it_.subscribe("/line_vision_node/vision/line/image_output",1,&lineStrategy::imageCallback,this);
    line_left_sub_=nh_.subscribe<line_vision::Line>("/line_vision_node/vision/line/leftline",1,&lineStrategy::leftCallback,this);
    line_right_sub_=nh_.subscribe<line_vision::Line>("/line_vision_node/vision/line/rightline",1,&lineStrategy::rightCallback,this);
    // walk_pub_=nh_.advertise<briker_strategy::WalkingParam>("walkParam",100);
    command_pub_=nh_.advertise<std_msgs::Int16>("command",100);
    command_pub_2=nh_.advertise<std_msgs::Int16>("command2",100);
    x_move_pub_=nh_.advertise<std_msgs::Int16>("walkParam_x_move",1000);
    y_move_pub_=nh_.advertise<std_msgs::Int16>("walkParam_y_move",1000);
    angle_pub_=nh_.advertise<std_msgs::Int16>("walkParam_angle",1000);
    x_speed_pub_=nh_.advertise<std_msgs::Int16>("walkParam_x_speed",1000);
    y_speed_pub_=nh_.advertise<std_msgs::Int16>("walkParam_y_speed",1000);
    angle_speed_pub_=nh_.advertise<std_msgs::Int16>("walkParam_angle_speed",1000);
    publisherCommand();
}

void lineStrategy::adjustToward()
{
    if(left.size()&&right.size())
    {
        //根据中线两点像素位置计算角度，这样显然是不准确的后面根据实际再调整
        m_top_center=Point((left[0].x+right[0].x)/2,(left[0].y+right[0].y)/2);
        m_bottom_center=Point((left[1].x+right[1].x)/2,(left[1].y+right[1].y)/2);
        double d=m_bottom_center.x-img_width/2;
        double m_target_x = m_top_center.x-d;
        double m_target_y = m_bottom_center.y-m_top_center.y;
        double m_degree = atan(m_target_x/m_target_y)*180/CV_PI;//角度
        ROS_INFO("m_target: x:%lf y:%lf",m_target_x,m_target_y);
        ROS_INFO("m_degree: %lf",m_degree);
        setWalkingParam(0,0,0,0,0,0);
    }else {
        ROS_INFO("error, no points");
    }
}

void lineStrategy::walkFar()
{
    setWalkingParam(0,0,0,0,0,0);
    usleep(300*1000);
    premode=WALK_FAR;
    process_mode=ADJUSTTOWARD;
}

void lineStrategy::walkAccurate()
{
    setWalkingParam(0,0,0,0,0,0);
    usleep(300*1000);
    premode=WALK_ACCURATE;
    process_mode=ADJUSTTOWARD;
}

 void lineStrategy::setWalkingParam(int x_move,int y_move,int angle,int x_speed,int y_speed,int angle_speed)
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

void lineStrategy::publisherCommand()
{
    std_msgs::Int16 msg;
    msg.data=1;
    command_pub_.publish(msg);
    msg.data=2;
    command_pub_2.publish(msg);
}
void lineStrategy::stop()
{
    
}

lineStrategy::~lineStrategy()
{

}
}