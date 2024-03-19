#ifndef MINEIMG_H
#define MINEIMG_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

#include"mine_vision/Rect.h"

using namespace std;
using namespace cv;

class MineDetector
{
public:
    MineDetector();
    void imageProcess();
    void publishMine();
protected:
    //初始化列表部分
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    cv::VideoCapture capture;
    //有关图像的部分成员
    image_transport::Publisher img_pub_;
    cv::Mat img_src_;
    cv::Mat img_out_;
    ros::Publisher target_pub;
    mine_vision::Rect target_rect_; 
    //第一部分目标颜色范围
    int thre_h_min_1;
    int thre_h_max_1;
    int thre_s_min_;
    int thre_s_max_;
    int thre_v_min_;
    int thre_v_max_;
    
    //第二部分目标颜色范围
    int thre_h_min_2;
    int thre_h_max_2;

    //Canny的两个阈值
    int thre_canny_1;
    int thre_canny_2;
};

#endif
