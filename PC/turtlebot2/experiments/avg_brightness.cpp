#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

cv::Mat prev_frame;
bool first_time;

void color_camera(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        if(enc::isColor(msg->encoding))
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else
            cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        cv::Mat rgb_img = cv_ptr->image;
        cv::imshow("rgb", rgb_img);
        cv::waitKey(1);
        Mat hsv_img;
        cvtColor(rgb_img, hsv_img, CV_BGR2HSV);
        vector<Mat> channel;
        split(hsv_img, channel);
        Scalar m = mean(channel[2]);
        printf("%f\n", m[0]);
    }catch(const cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    first_time = true;
    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1, color_camera);
    ros::spin();
}
