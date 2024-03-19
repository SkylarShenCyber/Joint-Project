#ifndef LINEIMG_H
#define LINEIMG_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

#include"line_vision/Line.h"

using namespace std;
using namespace cv;

class LineDetector
{
public:
    LineDetector();
    void imageProcess();
    void publishLine();
    void deleteleft(Mat input);
    void deleteright(Mat input);
    vector<Point> findMaxcontours(Mat input);
protected:
    //初始化列表部分
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    cv::VideoCapture capture;
    //有关图像的部分成员
    image_transport::Publisher img_pub_;
    image_transport::Publisher img_blured_left_pub_;
    image_transport::Publisher img_blured_right_pub_;
    cv::Mat img_src_;
    cv::Mat img_out_;
    cv::Mat img_blured_right_;
    cv::Mat img_blured_left_;

    ros::Publisher points_left_pub_;
    ros::Publisher points_right_pub_;
    line_vision::Line Lines[2];
    //第一部分红色范围
    int thre_h_min_1;
    int thre_h_max_1;
    int thre_s_min_;
    int thre_s_max_;
    int thre_v_min_;
    int thre_v_max_;
    
    //第二部分红色范围
    int thre_h_min_2;
    int thre_h_max_2;

    //Canny的两个阈值
    int thre_canny_1;
    int thre_canny_2;
};

#endif
