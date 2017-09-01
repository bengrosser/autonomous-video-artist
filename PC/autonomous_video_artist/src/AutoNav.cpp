/**********************
* A class for the autonomous video artist,
* This part in the constructor of the class.
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/

#include "AutoNav.h"

void my_handler(int sig)
{
    shutdown = true;
    ros::shutdown();
    cv::destroyAllWindows();
    Json::StyledWriter styledWriter;
    std::ofstream fid;
    fid.open("metadata.json");
    fid << styledWriter.write(jsonarray);
    fid.close();
}
 

AutoNav::AutoNav(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10))
{
    move_forward = true;
    bump = false;
    img_height = 480;
    img_width = 640;
    go_right = false;
    avoid_from_right = false;
    battery_is_low = false;
    battery_is_full = true;
    half_battery = false;
    in_charging = false;
    near_docking_station = false;
    leave_docking_station = true;
    acc_or_not = true;
    near_docking_station_x = -0.8;
    near_docking_station_y = 0.0;
    docking_station_x = 0.0;
    docking_station_y = 0.0;
    current_x = 0.0;
    current_y = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    motion = false;
    first_time = true;

    node.getParamCached("drive", DRIVE);
    node.getParamCached("drive_linearspeed", linear_speed);
    node.getParamCached("drive_angularspeed", angular_speed);

    signal(SIGINT, my_handler);

    ros::MultiThreadedSpinner threads(9);

    image_transport::ImageTransport it(node);
    image_transport::Subscriber frontEnv=it.subscribe("/camera/depth/image", 1, &AutoNav::frontEnv, this);
    
    ros::Timer pilot=node.createTimer(ros::Duration(0.1), &AutoNav::pilot, this);

    ros::Subscriber battery=node.subscribe("/mobile_base/sensors/core", 10, &AutoNav::battery, this);

    ros::Subscriber bumperCommand=node.subscribe("/mobile_base/events/bumper",10, &AutoNav::bumperStatus, this);

    ros::Subscriber sysInfo=node.subscribe("/sys/freeRAM", 1, &AutoNav::sysInfo, this);

    ros::Subscriber autoCharging=node.subscribe("/odom", 10, &AutoNav::angle, this);

    ros::Subscriber preAnalysis = node.subscribe("/camera/rgb/image_raw", 1, &AutoNav::preAnalysis, this);

    ros::Timer writeJson=node.createTimer(ros::Duration(1.0), &AutoNav::writeJson, this);

    threads.spin();
    
}
