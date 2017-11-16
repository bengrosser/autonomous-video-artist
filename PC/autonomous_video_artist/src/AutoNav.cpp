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
    printf("my handler\n");
    Json::StyledWriter styledWriter;
    std::ofstream fid;
    fid.open("metadata.json");
    printf("open json file\n");
    fid << styledWriter.write(jsonarray);
    printf("write done\n");
    cv::destroyAllWindows();
    shutdown = true;
    fid.close();
    ros::shutdown();
}
 

AutoNav::AutoNav(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10))
{
    move_forward = true;
    bump = false;
	which_bumper = 0;
    img_height = 480;
    img_width = 320;
    go_right = false;
	battery_is_low = false;
    battery_is_full = true;
    half_battery = false;
	battery_value = 162;
	in_charging = false;
	near_docking_station = false;
    leave_docking_station = true;
    avoid_from_right = false;    
    acc_or_not = true;


    near_docking_station_x = -0.8;
    near_docking_station_y = 0.0;
    docking_station_x = 0.0;
    docking_station_y = 0.0;
    current_x = 0.0;
    current_y = 0.0;
	distance_to_docking = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

	freeRAM = 0;
	RAM_in_use = 0;
	freeSwap = 0;
	Swap_in_use = 0;
	uptime = 0;

	
	bit = true;
	entropy = 0.0;
	brightness = 0.0;
	avg_front_distance = -1.0;
    motion = false;
    first_time = true;
    motion_map = cv::Mat::zeros(480,640, CV_32FC1);
    
    node.getParamCached("drive", DRIVE);
    node.getParamCached("drive_linearspeed", linear_speed);
    node.getParamCached("drive_angularspeed", angular_speed);

    signal(SIGINT, my_handler);

    ros::MultiThreadedSpinner threads(13);

    image_transport::ImageTransport it(node);
    image_transport::Subscriber frontEnv=it.subscribe("/camera/depth/image", 1, &AutoNav::frontEnv, this);

    image_transport::Subscriber preAnalysis=it.subscribe("/camera/rgb/image_raw", 1, &AutoNav::preAnalysis, this);

    /*ros::Subscriber preAnalysis = node.subscribe("/camera/rgb/image_raw", 1, &AutoNav::preAnalysis, this);

    ros::Subscriber frontEnv=node.subscribe("/camera/depth/image", 1, &AutoNav::frontEnv, this);*/
    
    ros::Timer pilot=node.createTimer(ros::Duration(0.1), &AutoNav::pilot, this);

    ros::Subscriber battery=node.subscribe("/mobile_base/sensors/core", 10, &AutoNav::battery, this);

    ros::Subscriber bumperCommand=node.subscribe("/mobile_base/events/bumper",10, &AutoNav::bumperStatus, this);

    ros::Subscriber sys_freeRAM=node.subscribe("/sys/freeRAM", 1, &AutoNav::sys_freeRAM, this);
	ros::Subscriber sys_totalRAM=node.subscribe("/sys/totalRAM", 1, &AutoNav::sys_totalRAM, this);
	ros::Subscriber sys_freeSwap=node.subscribe("/sys/freeSwap", 1, &AutoNav::sys_freeSwap, this);
	ros::Subscriber sys_totalSwap=node.subscribe("/sys/totalSwap", 1, &AutoNav::sys_totalSwap, this);
	ros::Subscriber sys_uptime=node.subscribe("/sys/uptime", 1, &AutoNav::sys_uptime, this);
	

    ros::Subscriber autoCharging=node.subscribe("/odom", 10, &AutoNav::angle, this);

    ros::Timer writeJson=node.createTimer(ros::Duration(1.0), &AutoNav::writeJson, this);

	ros::Timer panning = node.createTimer(ros::Duration(0.1), &AutoNav::video_control, this);

	//NO HEADER, DOESN't WORK!
	/*message_filters::Subscriber<Int32> sys_sub1(node, "sys/freeRAM", 1);
	message_filters::Subscriber<Int32> sys_sub2(node, "sys/totalRAM", 1);
	message_filters::Subscriber<Int32> sys_sub3(node, "sys/freeSwap", 1);
	message_filters::Subscriber<Int32> sys_sub4(node, "sys/totalSwap", 1);
	message_filters::Subscriber<Int32> sys_sub5(node, "sys/uptime", 1);
	TimeSynchronizer<Int32, Int32, Int32, Int32, Int32> sync(sys_sub1, sys_sub2, sys_sub3, sys_sub4, sys_sub5, 20);
	sync.registerCallback(boost::bind(&AutoNav::sysInfo, this, _1, _2, _3, _4, _5));*/

    threads.spin();
    
}
