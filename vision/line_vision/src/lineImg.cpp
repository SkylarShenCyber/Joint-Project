#include "lineImg.h"

LineDetector::LineDetector():
    nh_(ros::this_node::getName()),
    it_(this->nh_),
    capture(0)
{
    img_pub_=it_.advertise("vision/line/image_output",1);
    img_blured_left_pub_=it_.advertise("vision/line/image_left",1);
    img_blured_right_pub_=it_.advertise("vision/line/image_right",1);
    points_left_pub_=nh_.advertise<line_vision::Line>("vision/line/leftline",1000);
    points_right_pub_=nh_.advertise<line_vision::Line>("vision/line/rightline",1000);
    //获取外部参数
    ros::param::get("thre_h_min_1",thre_h_min_1);
    ros::param::get("thre_h_max_1",thre_h_max_1);
    ros::param::get("thre_s_min_",thre_s_min_);
    ros::param::get("thre_s_max_",thre_s_max_);
    ros::param::get("thre_v_min_",thre_v_min_);
    ros::param::get("thre_v_max_",thre_v_max_);
    ros::param::get("thre_h_min_2",thre_h_min_2);
    ros::param::get("thre_h_max_2",thre_h_max_2);
    ros::param::get("thre_canny_1",thre_canny_1);
    ros::param::get("thre_canny_2",thre_canny_2);
}

void LineDetector::deleteleft(Mat input)
{
    cv::Rect rect_mask(Point(0,0),Point(input.cols/2,input.rows));
    cv::Mat mask = input(rect_mask);
    mask.setTo(0);
    return;
    
}

void LineDetector::deleteright(Mat input)
{
    cv::Rect rect_mask(Point(input.cols/2,0),Point(input.cols,input.rows));
    cv::Mat mask = input(rect_mask);
    mask.setTo(0);
    return;
}

vector<Point> LineDetector::findMaxcontours(Mat input)
{
    std::vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(input,contours,hierarchy,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    std::vector<Point> maxcontours;
    for (size_t i = 0; i < contours.size(); i++)
    {
        if(contours[i].size()>maxcontours.size())
        {
            maxcontours=contours[i];
        }
    }
    return maxcontours;
}

void LineDetector::imageProcess()
{
    capture>>img_src_;
    capture>>img_out_;
    Mat img_hsv;
    cv::cvtColor(img_src_,img_hsv,COLOR_BGR2HSV);
    Mat img_red_(img_hsv.size(),img_hsv.type());
    Mat img_red_1_(img_red_.size(),img_red_.type());
    Mat img_red_2_(img_red_.size(),img_red_.type());
    cv::inRange(img_hsv,Scalar(thre_h_min_1,thre_s_min_,thre_v_min_),Scalar(thre_h_max_1,thre_s_max_,thre_v_max_),img_red_1_);
    cv::inRange(img_hsv,Scalar(thre_h_min_2,thre_s_min_,thre_v_min_),Scalar(thre_h_max_2,thre_s_max_,thre_v_max_),img_red_2_);
    cv::addWeighted(img_red_1_,0.5,img_red_2_,0.5,0,img_red_);
    cv::GaussianBlur(img_red_,img_red_,Size(3,3),0,0);
    Mat img_cannyed_;
    cv::Canny(img_red_,img_cannyed_,thre_canny_1,thre_canny_2);
    Mat img_blured_;
    cv::blur(img_cannyed_,img_blured_,Size(3,3));
    
    img_blured_right_=img_blured_.clone();
    img_blured_left_=img_blured_.clone();
    deleteleft(img_blured_right_);
    deleteright(img_blured_left_);
    
    vector<Point> leftline=findMaxcontours(img_blured_left_);
    vector<Point> rightline=findMaxcontours(img_blured_right_);

    Point bottom1(0,0);
    for (size_t i = 0; i < leftline.size(); i++)
    {
        if(leftline[i].y>bottom1.y)
        {
            bottom1=leftline[i];
        }
    }
    Point top1=bottom1;
    for (size_t i = 0; i < leftline.size(); i++)
    {
        if(leftline[i].y<top1.y)
        {
            top1=leftline[i];
        }
    }
    Point bottom2(0,0);
    for (size_t i = 0; i < rightline.size(); i++)
    {
        if(rightline[i].y>bottom2.y)
        {
            bottom2=rightline[i];
        }
    }
    Point top2=bottom2;
    for (size_t i = 0; i < rightline.size(); i++)
    {
        if(rightline[i].y<top2.y)
        {
            top2=rightline[i];
        }
    }
    cv::line(img_out_,top1,bottom1,Scalar(255,0,0),3);
    cv::line(img_out_,top2,bottom2,Scalar(0,255,0),3);
    Lines[0].topPoint.x=top1.x;
    Lines[0].topPoint.y=top1.y;
    Lines[0].bottomPoint.x=bottom1.x;
    Lines[0].bottomPoint.y=bottom1.y;

    Lines[1].topPoint.x=top2.x;
    Lines[1].topPoint.y=top2.y;
    Lines[1].bottomPoint.x=bottom2.x;
    Lines[1].bottomPoint.y=bottom2.y;
}

void LineDetector::publishLine()
{
    sensor_msgs::ImagePtr msg;
    msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_out_).toImageMsg();
    img_pub_.publish(msg);
    sensor_msgs::ImagePtr msg_left;
    msg_left=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_blured_left_).toImageMsg();
    sensor_msgs::ImagePtr msg_right;
    msg_left=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_blured_right_).toImageMsg();
    points_left_pub_.publish(Lines[0]);
    points_right_pub_.publish(Lines[1]);
    usleep(400*1000);
}

