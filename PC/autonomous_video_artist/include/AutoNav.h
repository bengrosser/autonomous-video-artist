/**********************
* A class for the autonomous video artist
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/
#ifndef AUTONAV_H_
#define AUTONAV_H_

#include <iostream>
#include <vector>
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
#include <std_msgs/Int32.h>
#include <kobuki_msgs/SensorState.h>
#include <sys/sysinfo.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <signal.h>
#include <thread>
#include <chrono>
#include <jsoncpp/json/writer.h>
#include <jsoncpp/json/json.h>
#include <fstream>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

extern bool shutdown;
extern const double pi;
extern const uint8_t MAX_BATTERY;
extern Json::Value jsonarray;
void my_handler(int sig);

class AutoNav{
private:
    ros::NodeHandle node;
    ros::Publisher velocity;
    bool move_forward;
    bool bump;
    int which_bumper;
    int img_height;
    int img_width;
    bool go_right;
    bool battery_is_low;
    bool battery_is_full;
    bool half_battery;
    int battery_value;
    bool in_charging;
    bool near_docking_station;
    bool leave_docking_station;
    bool avoid_from_right;
    bool acc_or_not;
    

    double near_docking_station_x;
    double near_docking_station_y;
    double docking_station_x;
    double docking_station_y;
    double current_x;
    double current_y;
    double distance_to_docking;
    float roll;
    float pitch;
    float yaw;
    int view_height;
    int view_bottom;

    bool DRIVE;
    double linear_speed;
    double angular_speed;

    bool bit;
    double entropy;
    double brightness;
    double avg_front_distance;
    
    

    //Preliminary analysis
    vector<double> colorPercent(const cv_bridge::CvImageConstPtr cv_ptr, int group_num);
    unsigned int count_bits(int n);
    bool bitAnalysis(const cv::Mat rgb_img);
    void preAnalysis(const sensor_msgs::ImageConstPtr& msg);
    double avg_distance(const cv::Mat depth_img);
    double image_entropy(const cv::Mat image);
    double avg_brightness(const cv::Mat image);
    //take videos
    void webcam0(const sensor_msgs::ImageConstPtr& msg);
    void webcam1(const sensor_msgs::ImageConstPtr& msg);
    //vision analysis
    void frontEnv(const sensor_msgs::ImageConstPtr& msg);
    //navigation
    float acc_speed(double target_velocity, double duration, double time_elapsed);
    float dec_speed(double start_velocity, double duration, double time_elapsed);
    void linear_accelerate(const ros::TimerEvent& time, double target_velocity, double duration);
    void angular_accelerate(const ros::TimerEvent& time, double target_velocity, double duration);  
    void linear_decelerate(const ros::TimerEvent& time, double start_velocity, double duration);
    void angular_decelerate(const ros::TimerEvent& time, double start_velocity, double duration);
    void leave_station_action(const ros::TimerEvent& time);
    void battery_is_good_action(const ros::TimerEvent& time);
    void auto_docking_action(const ros::TimerEvent& time);
    void battery_is_low_action(const ros::TimerEvent& time);
    void pilot(const ros::TimerEvent& time);
    //machine status
    void bumperStatus(const kobuki_msgs::BumperEvent msg);
    void battery(const kobuki_msgs::SensorState msg);
    void sysInfo(const std_msgs::Int32::ConstPtr& msg);
    void angle(const nav_msgs::Odometry::ConstPtr& msg);
    void toEulerianAngle(const float x, const float y, const float z, const float w, float& roll, float& pitch, float& yaw);
    void writeJson(const ros::TimerEvent& time);
    

public:
    AutoNav(ros::NodeHandle& handle);   
};

#endif
