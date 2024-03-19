#include "squareImg.h"
#include <vector>

SquareDetector::SquareDetector():
    nh_(ros::this_node::getName()),
    it_(this->nh_),
    capture(0)
{
    img_pub_=it_.advertise("vision/square/image_output",1);
    square_info_pub=nh_.advertise<square_vision::Square>("vision/square/square_info",20);
    destination_info_pub=nh_.advertise<square_vision::Destination>("vision/square/destination_info",20);
    //获取外部参数
    ros::param::get("thre_h_min_1",square_h_min_1);
    ros::param::get("thre_h_max_1",square_h_max_1);
    ros::param::get("thre_s_min_",square_s_min_);
    ros::param::get("thre_s_max_",square_s_max_);
    ros::param::get("thre_v_min_",square_v_min_);
    ros::param::get("thre_v_max_",square_v_max_);
    ros::param::get("thre_h_min_2",square_h_min_2);
    ros::param::get("thre_h_max_2",square_h_max_2);
    ros::param::get("thre_canny_1",thre_canny_1);
    ros::param::get("thre_canny_2",thre_canny_2);
    ros::param::get("destination_h_min_1",destination_h_min_1);
    ros::param::get("destination_h_max_1",destination_h_max_1);
    ros::param::get("destination_h_min_2",destination_h_min_2);
    ros::param::get("destination_h_max_2",destination_h_max_2);
    ros::param::get("destination_s_min",destination_s_min);
    ros::param::get("destination_s_max",destination_s_max);
    ros::param::get("destination_v_min",destination_v_min);
    ros::param::get("destination_v_max",destination_v_max);

    //square和destination初始化
    square.upleftPoint.x=-1;
    square.upleftPoint.y=-1;
    square.downrightPoint.x=-1;
    square.downrightPoint.y=-1;
    square.center.x=-1;
    square.center.y=-1;
    square.valid=true;
    
    destination.upleftPoint.x=-1;
    destination.upleftPoint.y=-1;
    destination.downrightPoint.x=-1;
    destination.center.x=-1;
    destination.center.y=-1;
    destination.valid=false;
}

void SquareDetector::imageProcess()
{
    capture>>img_src_;
    capture>>img_out_;
    if(square.valid==true)
    {
        detectorSquare();
    }
    else if(destination.valid==true)
    {
        detectorDestination();
    }
}

void SquareDetector::detectorSquare()
{
    Mat img_hsv;
    cvtColor(img_src_,img_hsv,COLOR_BGR2HSV);
    Mat img_red_(img_hsv.size(),img_hsv.type());
    Mat img_red_1_(img_red_.size(),img_red_.type());
    Mat img_red_2_(img_red_.size(),img_red_.type());
    inRange(img_hsv,Scalar(square_h_min_1,square_s_min_,square_v_min_),Scalar(square_h_max_1,square_s_max_,square_v_max_),img_red_1_);
    inRange(img_hsv,Scalar(square_h_min_2,square_s_min_,square_v_min_),Scalar(square_h_max_2,square_s_max_,square_v_max_),img_red_2_);
    addWeighted(img_red_1_,0.5,img_red_2_,0.5,0,img_red_);
    GaussianBlur(img_red_,img_red_,Size(3,3),0,0);
    Mat img_cannyed_;
    Canny(img_red_,img_cannyed_,thre_canny_1,thre_canny_2);
    Mat img_blured_;
    blur(img_cannyed_,img_blured_,Size(3,3));

    //轮廓检测
    std::vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(img_blured_,contours,hierarchy,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    //框出目标
    vector<RotatedRect> m_Rect(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        m_Rect[i]=minAreaRect(Mat(contours[i]));
    }
    //选出最大的矩形
    RotatedRect maxRect;
    if(m_Rect.size()!=0)
    {
        maxRect=m_Rect[0];
    }
    for (int i = 0; i < m_Rect.size(); i++)
    {
        if(maxRect.size.area() < m_Rect[i].size.area())
        {
            maxRect=m_Rect[i];
        }
    }

        Point2f vtx[4];
        maxRect.points(vtx);
        square.upleftPoint.x = vtx[1].x;
        square.upleftPoint.y = vtx[1].y;
        square.downrightPoint.x = vtx[3].x;
        square.downrightPoint.y = vtx[3].y;
        square.center.x = maxRect.center.x;
        square.center.y = maxRect.center.y;

    //画出矩形
    cv::rectangle(img_out_,vtx[1],vtx[3],Scalar(255,0,0),2);
}

void SquareDetector::detectorDestination()
{
    Mat img_hsv;
    cvtColor(img_src_,img_hsv,COLOR_BGR2HSV);
    Mat img_red_(img_hsv.size(),img_hsv.type());
    Mat img_red_1_(img_red_.size(),img_red_.type());
    Mat img_red_2_(img_red_.size(),img_red_.type());
    inRange(img_hsv,Scalar(destination_h_min_1,destination_s_min,destination_v_min),Scalar(destination_h_max_1,destination_s_max,destination_v_max),img_red_1_);
    inRange(img_hsv,Scalar(destination_h_min_2,destination_s_min,destination_v_min),Scalar(destination_h_max_2,destination_s_max,destination_v_max),img_red_2_);
    addWeighted(img_red_1_,0.5,img_red_2_,0.5,0,img_red_);
    GaussianBlur(img_red_,img_red_,Size(3,3),0,0);
    Mat img_cannyed_;
    Canny(img_red_,img_cannyed_,thre_canny_1,thre_canny_2);
    Mat img_blured_;
    blur(img_cannyed_,img_blured_,Size(3,3));

    //轮廓检测
    std::vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(img_blured_,contours,hierarchy,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    //框出目标
    vector<RotatedRect> m_Rect(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        m_Rect[i]=minAreaRect(Mat(contours[i]));
    }
    //选出最大的矩形
    RotatedRect maxRect;
    if(m_Rect.size()!=0)
    {
        maxRect=m_Rect[0];
    }
    for (int i = 0; i < m_Rect.size(); i++)
    {
        if(maxRect.size.area() < m_Rect[i].size.area())
        {
            maxRect=m_Rect[i];
        }
    }

        Point2f vtx[4];
        maxRect.points(vtx);
        destination.upleftPoint.x = vtx[1].x;
        destination.upleftPoint.y = vtx[1].y;
        destination.downrightPoint.x = vtx[3].x;
        destination.downrightPoint.y = vtx[3].y;
        destination.center.x = maxRect.center.x;
        destination.center.y = maxRect.center.y;
    //画出矩形
    cv::rectangle(img_out_,vtx[1],vtx[3],Scalar(255,0,0),2);
    
}

void SquareDetector::publishInfo()
{
    if(square.valid==true)
    {
        publishSquare();
    }
    else if(destination.valid==true)
    {
        publishDestination();
    }
}

void SquareDetector::publishSquare()
{
    //发布处理后的图像
    sensor_msgs::ImagePtr msg;
    msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_out_).toImageMsg();
    img_pub_.publish(msg);
    square_info_pub.publish(square);
    usleep(400*1000);
}

void SquareDetector::publishDestination()
{
    //发布处理后的图像
    sensor_msgs::ImagePtr msg;
    msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_out_).toImageMsg();
    img_pub_.publish(msg);
    destination_info_pub.publish(destination);
    usleep(400*1000);
}