#ifndef LINESTRATEGY_H
#define LINESTRATEGY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <boost/bind.hpp>
#include <vector>

#include "briker_strategy/WalkingParam.h"
#include "line_vision/Line.h"

#define _CATCH 1
#define _UP 2
#define _DOWN 3

using namespace std;
using namespace cv;

namespace line{
class lineStrategy
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    ros::Subscriber line_left_sub_;
    ros::Subscriber line_right_sub_;
    ros::Publisher walk_pub_;
    ros::Publisher command_pub_;
    ros::Publisher command_pub_2;
    ros::Publisher x_move_pub_;
    ros::Publisher y_move_pub_;
    ros::Publisher angle_pub_;
    ros::Publisher x_speed_pub_;
    ros::Publisher y_speed_pub_;
    ros::Publisher angle_speed_pub_;

    std_msgs::Int16 command;
    std_msgs::Int16 command2;
    int img_width,img_height;
    vector<Point> left,right;
    //通过传入的数据得到的梯形上下边线的中点
    Point m_top_center,m_bottom_center;
    //
    bool enable_;
public:
    enum State
    {
        WALK_FAR,
        WALK_ACCURATE,
        ADJUSTTOWARD,
        STOP
    };
    //小车当前的模式
    int process_mode,premode;
    int dis;
    //自定消息
    briker_strategy::WalkingParam next_move;
    //
    lineStrategy();
    ~lineStrategy();
    //主进程
    void process();
    //打印左右两条线的信息
    void testINFO();
    //回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightCallback(const line_vision::Line::ConstPtr& msg);
    void leftCallback(const line_vision::Line::ConstPtr& msg);
    //
    void startMode();
    //小车动作
    void walkFar();
    void walkAccurate();
    void adjustToward();
    void stop();
    //设置小车动作
    void setWalkingParam(int x_move,int y_move,int angle,int x_speed,int y_speed,int angle_speed);
    void setArmMode(int mode);
    //发布者
    void publisherCommand();
    
};
}

#endif
