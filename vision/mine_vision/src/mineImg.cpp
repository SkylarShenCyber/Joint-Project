#include "mineImg.h"

MineDetector::MineDetector():
    nh_(ros::this_node::getName()),
    it_(this->nh_),
    capture(0)
{
    img_pub_=it_.advertise("image_output",1);
    target_pub=nh_.advertise<mine_vision::Rect>("target_info",20);
    
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

void MineDetector::imageProcess()
{
    capture>>img_src_;
    capture>>img_out_;
    Mat img_hsv;
    cvtColor(img_src_,img_hsv,COLOR_BGR2HSV);
    Mat img_red_(img_hsv.size(),img_hsv.type());
    Mat img_red_1_(img_red_.size(),img_red_.type());
    Mat img_red_2_(img_red_.size(),img_red_.type());
    inRange(img_hsv,Scalar(thre_h_min_1,thre_s_min_,thre_v_min_),Scalar(thre_h_max_1,thre_s_max_,thre_v_max_),img_red_1_);
    inRange(img_hsv,Scalar(thre_h_min_2,thre_s_min_,thre_v_min_),Scalar(thre_h_max_2,thre_s_max_,thre_v_max_),img_red_2_);
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

    //目标矩形左上点和右下点
    Point2f vtx[4];
    maxRect.points(vtx);
    target_rect_.upleftPoint.x = vtx[1].x;
    target_rect_.upleftPoint.y = vtx[1].y;
    target_rect_.downrightPoint.x = vtx[3].x;
    target_rect_.downrightPoint.y = vtx[3].y;
    
    //画出矩形
    cv::rectangle(img_out_,vtx[1],vtx[3],Scalar(255,0,0),2);
}

void MineDetector::publishMine()
{
    sensor_msgs::ImagePtr msg;
    msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_out_).toImageMsg();
    img_pub_.publish(msg);
    target_pub.publish(target_rect_);
    // ROS_INFO("%f %f",target_rect_.upleftPoint.x,target_rect_.upleftPoint.y);
    usleep(400*1000);
}