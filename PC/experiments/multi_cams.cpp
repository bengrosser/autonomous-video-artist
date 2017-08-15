#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <sys/sysinfo.h> //for system infomation
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <signal.h>
#include <thread>
#include <chrono>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class Cameras{
private:
    ros::NodeHandle node;
    int cam0_video_idx;
    int cam1_video_idx;
    string filename0;
    string filename1;
    VideoWriter video0;
    VideoWriter video1;
    bool take_video0;
    bool take_video1;
    bool change_par0;
    bool change_par1;
    bool release0;
    bool release1;
    bool shoot0;
    bool shoot1;

public:
    Cameras(ros::NodeHandle& handle):node(handle), cam0_video_idx(0), cam1_video_idx(0), take_video0(false), take_video1(false), change_par0(true), change_par1(true), shoot0(false), shoot1(false){
        video0.release();
        video1.release();

        //video0.open("camer0.avi", CV_FOURCC('M','J','P','G'),22, Size(1920, 1080), true);
        //video1.open("camer1.avi", CV_FOURCC('M','J','P','G'),22, Size(1920, 1080), true);

        ros::MultiThreadedSpinner threads(4);

        ros::Timer control=node.createTimer(ros::Duration(0.01), &Cameras::control, this);

        ros::Subscriber odom=node.subscribe("/odom", 1, &Cameras::odom, this);

        //This part of code is for /image_raw
        /*ros::Subscriber record0=node.subscribe("/webcam0/usb_cam0/image_raw", 1, &Cameras::record0, this);
        
        ros::Subscriber record1=node.subscribe("/webcam1/usb_cam1/image_raw", 1, &Cameras::record1, this);*/

        //This part of code is for /image_raw/compressed
        image_transport::ImageTransport it(node);
        image_transport::Subscriber record0=it.subscribe("/webcam0/usb_cam0/image_raw", 1, &Cameras::record0, this);

        
        image_transport::Subscriber record1=it.subscribe("/webcam1/usb_cam1/image_raw", 1, &Cameras::record1, this);
        
        threads.spin();
    }

    void control(const ros::TimerEvent& time){
        if(take_video0){
            if(change_par0){
                filename0 = "cam0video"+to_string(cam0_video_idx)+".avi";
                video0.open(filename0, CV_FOURCC('M','J','P','G'),30, Size(640, 480), true);
                change_par0 = false;
                ++cam0_video_idx;
            }
            shoot0 = true;
        }
        else{
            shoot0 = false;
        }
        if(take_video1){
            if(change_par1){
                filename1 = "cam1video"+to_string(cam1_video_idx)+".avi";
                video1.open(filename1, CV_FOURCC('M','J','P','G'),30, Size(640, 480), true);
                change_par1 = false;
                ++cam1_video_idx;
            }    
            shoot1 = true;
        }
        else{
            shoot1 = false;
        }
    }

    void odom(const nav_msgs::Odometry::ConstPtr& msg){
        std::cout<<"odom"<<std::endl;
        ros::Time start = ros::Time::now();
        while(ros::Time::now()-start < ros::Duration(2.0)){

        }
        change_par0 = true;
        take_video0 = true;
        change_par1 = true;
        take_video1 = true;
        start = ros::Time::now();
        while(ros::Time::now()-start < ros::Duration(5.0)){
            
        }
        release0 = true;
        take_video0 = false;
        release1 = true;
        take_video1 = false;
        start = ros::Time::now();
        while(ros::Time::now()-start < ros::Duration(5.0)){

        }
    }

    void record0(const sensor_msgs::ImageConstPtr& msg){
        /*cv_bridge::CvImageConstPtr cv_ptr;
        try{
            if(enc::isColor(msg->encoding))
                cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
            else
                cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
            cv::Mat rgb_img = cv_ptr ->image;
            video0.write(rgb_img);
        } catch (const cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }*/        

        std::cout<<"record0"<<std::endl;
        if(shoot0){
            cv_bridge::CvImageConstPtr cv_ptr;
            try{
                if(enc::isColor(msg->encoding))
                    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
                else
                    cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
                cv::Mat rgb_img = cv_ptr -> image;
                cv::imshow("webcam0", rgb_img);
                cv::waitKey(1);
                video0.write(rgb_img);
            } catch (const cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }
        else{
            if(release0){
                video0.release();
                release0 = false;
            }
        }
    }

    void record1(const sensor_msgs::ImageConstPtr& msg){
        std::cout<<"record1"<<std::endl;
        if(shoot1){
            cv_bridge::CvImageConstPtr cv_ptr;
            try{
                if(enc::isColor(msg->encoding))
                    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
                else
                    cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
                cv::Mat rgb_img = cv_ptr -> image;
                cv::imshow("webcam1", rgb_img);
                cv::waitKey(1);
                video1.write(rgb_img);
            } catch (const cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }
        else{
            if(release1){
                video1.release();
                release1 = false;
            }
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "test_cam");
    ros::NodeHandle node("cameras");
    Cameras turtlebot(node);
    return 0;
}
