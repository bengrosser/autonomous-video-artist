#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

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
        //cv::Mat::zeros diff(640, 480, CV_32FC1);
        cv::Mat mask1 = cv::Mat::ones(640, 480, CV_8U);
        cv::Mat mask2 = cv::Mat::ones(640, 480, CV_8U);

        /*if(first_time){
            prev_frame = depth_img;
            first_time = false;
        }*/
        if(count_num < 5)
        {
            ++count_num;
            prev_frame = depth_img;
        }
        else{
            count_num = 0;
            mask1 = prev_frame > 0;
            mask2 = depth_img > 0;
            diff = depth_img-prev_frame;
            prev_frame = depth_img;
            
            cv::imshow("depth_img", depth_img);
        double min = 0.0;
        double max = 0.0;
        cv::Mat mask;
        bitwise_and(mask1, mask2, mask);
        //if(!first_time)
        //cv::minMaxLoc(diff, &min, &max, 0, 0, mask);
        cv::minMaxLoc(diff, &min, &max);
        printf("max value in the image: %f\n", max);
        printf("min value in the iamge: %f\n", min);
        cv::waitKey(1);
        cv::Mat norm;
        diff.convertTo(norm, CV_32F, 1.0/max, 0);
        cv::imshow("norm", norm);
        cv::waitKey(1);
        norm.convertTo(norm, CV_8U, 255.0);
        video.write(norm);
        }
        
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
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
    ros::Subscriber sub = n.subscribe("/camera/depth/image", 10, recorder);
    ros::spin();
    video.release();
    return 0;
}
