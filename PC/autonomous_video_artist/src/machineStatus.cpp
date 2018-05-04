/**********************
* A class for the autonomous video artist,
* This part focus on the getting the status from the PC and the robot.
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/

#include "AutoNav.h"

const uint8_t MAX_BATTERY = 162;
Json::Value jsonarray;

void AutoNav::bumperStatus(const kobuki_msgs::BumperEvent msg)
{
    if(msg.state)
    {
        bump = true;
        which_bumper = msg.bumper;
    }
}

void AutoNav::battery(const kobuki_msgs::SensorState msg)
{
    if(!in_charging)
    {
        ros::Time start = ros::Time::now();
        while(ros::Time::now()-start < ros::Duration(5.0)){}
        battery_value = msg.battery;
        float percentage = ((float) msg.battery)/((float)MAX_BATTERY)*100.00;
        if(percentage <= 50)
            half_battery = true;
        if(percentage < 30)
        {
            battery_is_low = true;
            battery_is_full = false;
        }

    }
    else
    {
        ros::Time start = ros::Time::now();
        if(msg.battery >= 162)
        {
            leave_docking_station = true;
            battery_is_full = true;
            in_charging = false;
            battery_is_low = false;
            half_battery = false;
            near_docking_station = false;
        }
    }
}

/*void AutoNav::sysInfo(const std_msgs::Int32::ConstPtr& msg)
{
    freeRAM = msg->data;
    //ROS_INFO("Free RAM %d", ram);
}*/

/*void AutoNav::sysInfo(const std_msgs::Int32::ConstPtr& msg1, const std_msgs::Int32::ConstPtr& msg2, const std_msgs::Int32::ConstPtr& msg3, const std_msgs::Int32::ConstPtr& msg4, const std_msgs::Int32::ConstPtr& msg5)
{
	
}*/

void AutoNav::sys_freeRAM(const std_msgs::UInt64::ConstPtr& msg)
{
	freeRAM = msg->data;
}

void AutoNav::sys_totalRAM(const std_msgs::UInt64::ConstPtr& msg)
{
	RAM_in_use = msg->data;
}

void AutoNav::sys_freeSwap(const std_msgs::UInt64::ConstPtr& msg)
{
	freeSwap = msg->data;
}

void AutoNav::sys_totalSwap(const std_msgs::UInt64::ConstPtr& msg)
{
	Swap_in_use = msg->data;
}

void AutoNav::sys_uptime(const std_msgs::Int64::ConstPtr & msg)
{
	uptime = msg->data;
}

void AutoNav::angle(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    distance_to_docking = sqrt(pow(current_x, 2.0)+pow(current_y, 2.0)); 
    if(battery_is_low == true && distance_to_docking < 1.5)
        near_docking_station = true;

    float x = msg->pose.pose.orientation.x;
    float y = msg->pose.pose.orientation.y;
    float z = msg->pose.pose.orientation.z;
    float w = msg->pose.pose.orientation.w;
    toEulerianAngle(x,y,z,w,roll, pitch, yaw);
}

/* A function to convert quaternion angle to euler angle. */
void AutoNav::toEulerianAngle(const float x, const float y, const float z, const float w, float& roll, float& pitch, float& yaw)
{
    float ysqr = y*y;
    //roll (x-axis rotation)
    float t0 = +2.0*(w*x+y*z);
    float t1 = +1.0-2.0*(x*x+ysqr);
    roll = std::atan2(t0, t1);

    //pitch (y-axis rotation)
    float t2 = +2.0*(w*y-z*x);
    t2 = t2>1.0 ? 1.0 : t2;
    t2 = t2<-1.0 ? -1.0 : t2;
    pitch = std::asin(t2);

    //yaw (z-axis rotation)
    float t3 = +2.0*(w*z+x*y);
    float t4 = +1.0-2.0*(ysqr+z*z);
    yaw = std::atan2(t3, t4);

    //std::cout<<"roll: "<<roll<<std::endl;
    //std::cout<<"pitch: "<<pitch<<std::endl;
    //std::cout<<"yaw: "<<yaw<<std::endl;
}

double AutoNav::angle_converter(const double yaw){
    if(yaw > 0)
        return yaw/pi*180.0;
    else
        return 360.0-abs(yaw)/pi*180.0;
}
