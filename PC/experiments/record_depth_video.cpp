#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;

VideoWriter video;
cv::Mat prev_frame;
bool first_time;
int count_num;

void recorder(const sensor_msgs::ImageConstPtr& msg)
{
    //std::cout<<"recorder"<<std::endl;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
        cv::Mat depth_img;
        const std::string& enc = msg->encoding;
        if(enc.compare("16UC1") == 0){
            depth_img = cv_bridge::toCvShare(msg) -> image;
            //printf("16UC1\n");
        }
        else if(enc.compare("32FC1") == 0){
            cv_ptr->image.convertTo(depth_img, CV_16UC1, 1000,0);
            //printf("32FC1\n");
        }
        cv::Mat diff = cv::Mat::zeros(640, 480, CV_32FC1);
        cv::Mat diff_after = cv::Mat::zeros(640,480,CV_32FC1);

        if(first_time){
            prev_frame = depth_img;
            first_time = false;
        }
        else
        {
            cv::Mat mask1 = prev_frame > 0;
            cv::Mat mask2 = depth_img > 0;
            cv::Mat mask3;
            bitwise_and(mask1, mask2, mask3);
            cv::absdiff(depth_img, prev_frame, diff);
            mask1 = diff < 5000;
            bitwise_and(mask1, mask3, mask2);
            mask3 = diff > 1000;
            bitwise_and(mask2, mask3, mask1);
            cv::Mat norm;
            mask1.convertTo(norm, CV_8U, 255.0);           

            //blur(mask1, mask1, Size(6,6));
            blur(norm, norm, Size(10,10));
            blur(norm, norm, Size(10,10));
            cv::Mat new_mask = norm>180;
            cv::imshow("norm", norm);
            cv::imshow("mask", new_mask);
            cv::waitKey(1);
            
            prev_frame = depth_img;
            

            
            //video.write(norm);
        }
        
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void pilot(const ros::TimerEvent& time)
{
    ros::Publisher velocity;
    geometry_msgs::Twist decision;
    decision.linear.x = 0.5;
    decision.angular.z = 0.0;
    while(ros::ok())
    {
        velocity.publish(decision);
    }
}

int main(int argc, char** argv)
{
    first_time = true;
    count_num = 0;
    //video.open("no_motion.avi", CV_FOURCC('D','I','B',' '), 10, Size(640,480), false);
    video.open("no_motion.avi", CV_FOURCC('M','J','P','G'), 1, Size(640,480), false);
    if(!video.isOpened())
        printf("Not opened!\n");
    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/camera/depth/image", 10, recorder);
    ros::spin();
    /*ros::MultiThreadedSpinner threads(2);
    ros::Subscriber sub1 = n.subscribe("/camera/depth/image", 10, recorder);
    ros::Timer sub2=n.createTimer(ros::Duration(0.1), pilot);
    threads.spin();*/
    video.release();
    return 0;
}
