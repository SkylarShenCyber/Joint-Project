#ifndef MINEIMG_H
#define MINEIMG_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

#include "square_vision/Square.h"
#include "square_vision/Destination.h"

using namespace std;
using namespace cv;

class SquareDetector
{
public:
    SquareDetector();
    void imageProcess();
    void detectorSquare();
    void detectorDestination();
    void publishInfo();
    void publishSquare();
    void publishDestination();
protected:
    //初始化列表部分
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    cv::VideoCapture capture;
    
    //有关图像的部分成员
    image_transport::Publisher img_pub_;
    ros::Publisher square_info_pub;
    ros::Publisher destination_info_pub;
    cv::Mat img_src_;
    cv::Mat img_out_;

    square_vision::Square square;
    square_vision::Destination destination;

    //待抓取物的值
    //第一部分颜色范围
    int square_h_min_1;
    int square_h_max_1;
    int square_s_min_;
    int square_s_max_;
    int square_v_min_;
    int square_v_max_;
    
    //第二部分颜色范围
    int square_h_min_2;
    int square_h_max_2;

    //目的地的值
    //第一部分颜色范围
    int destination_h_min_1;
    int destination_h_max_1;
    //第二部分颜色范围
    int destination_h_min_2;
    int destination_h_max_2;
    //通用部分
    int destination_s_min;
    int destination_s_max;
    int destination_v_min;
    int destination_v_max;

    //Canny的两个阈值
    int thre_canny_1;
    int thre_canny_2;
};

#endif