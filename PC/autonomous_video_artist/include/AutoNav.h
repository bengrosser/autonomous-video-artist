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
#include <ctime>

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
    int freeRAM;
	int RAM_in_use;

    bool DRIVE;
    double static_linear_speed;
    double static_angular_speed;
    double linear_speed;  //real time speed
    double angular_speed;  //real time speed

    bool bit;
    double entropy;
    double brightness;
    double avg_front_distance;
    bool motion;
    bool first_time;
    Mat prev_frame;
    Mat motion_map;
    
	int video_clip;

    //Preliminary analysis
    void preAnalysis(const sensor_msgs::ImageConstPtr& msg);
    vector<double> colorPercent(const cv_bridge::CvImageConstPtr cv_ptr, int group_num);
    unsigned int count_bits(int n);
    bool bitAnalysis(const cv::Mat rgb_img);
    double avg_distance(const cv::Mat depth_img);
    double image_entropy(const cv::Mat rgb_img);
    double avg_brightness(const cv::Mat rgb_img);
    bool motion_detection(const cv::Mat depth_img);

    //take videos
	void video_control(const ros::TimerEvent& time);
	void shoot_video(const ros::TimerEvent& time);
	void panning(const ros::TimerEvent& time, double duration, double velocity);
	void camera(const ros::TimerEvent& time, string duration, string output_file_name, int camera_idx);
    void CorrespondingJson(string filename);

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
    double angle_converter(const double yaw);
    void writeJson(const ros::TimerEvent& time);
    

public:
    AutoNav(ros::NodeHandle& handle);   
};

#endif
