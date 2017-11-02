/**********************
* A class for the autonomous video artist,
* This part focus on taking videos.
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/

#include "AutoNav.h"

/*void AutoNav::webcam0(const sensor_msgs::ImageConstPtr& msg)
{

}

void AutoNav::webcam1(const sensor_msgs::ImageConstPtr& msg)
{

}*/
void AutoNav::video_control(const ros::TimerEvent& time)
{
	//if(rule) -> shoot video
		//shoot_video(time)
}

void AutoNav::shoot_video(const ros::TimerEvent& time)
{
	int hour = 0;
	int minute = 0;
	int second = 20;
	double speed = 1.0;
	int camera_idx = 0;
	//there should be a rule to define these parameters
	//TODO

	time_t t = std::time(0);
    struct tm * now = localtime(&t);
    string timestamp = to_string(now->tm_year + 1900)+"-"+to_string(now->tm_mon+1)+"-"+to_string(now->tm_mday)+"_"+to_string(now->tm_hour)+":"+to_string(now->tm_min)+":"+to_string(now->tm_sec);

	//Write a json file at the start of video shooting
	//CorrespondingJson(time, )  a json file at the start of shooting
	string json_filename = timestamp+"_start.json";
	CorrespondingJson(json_filename);

	//Take the video
	string video_filename = timestamp+".avi";
	string duration_str = to_string(hour)+":"+to_string(minute)+":"+to_string(second);
	camera(time, duration_str, video_filename, camera_idx);

	//Panning
	double duration = hour*3600+minute*60+second+5;
	panning(time, duration, speed);

	//Write a json file at the end of video shooting
	json_filename = timestamp+"_end.json";
	CorrespondingJson(json_filename);
}

void AutoNav::panning(const ros::TimerEvent& time, double duration, double speed)
{
	geometry_msgs::Twist decision;
	decision.linear.x = speed;
	decision.angular.z = 0.0;
	ros::Time current_time = ros::Time::now();
	while(ros::Time::now()- current_time <= ros::Duration(duration))
	{
		velocity.publish(decision);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void AutoNav::camera(const ros::TimerEvent& time, string duration, string output_file_name, int camera_idx)
{
	string command = "ffmpeg -f v4l2 -framerate 40 -video_size 640*480 -i /dev/video0 -t "+ duration + " " + output_file_name;
	const char* com = command.c_str(); 
	system(com);
	
}

void AutoNav::CorrespondingJson(string filename)
{
	Json::Value current_status;
	current_status["avg_brightness"] = brightness;
	current_status["avg_distance"] = avg_front_distance;
	current_status["motion_detected"] = motion;
	current_status["entropy"] = entropy;
	current_status["has_obstacle"] = !move_forward;
	current_status["position_X"] = current_x;
    current_status["position_Y"] = current_y;
	time_t t = std::time(0);
    struct tm * now = localtime(&t);
    string timestamp = to_string(now->tm_year + 1900)+"-"+to_string(now->tm_mon+1)+"-"+to_string(now->tm_mday)+" "+to_string(now->tm_hour)+":"+to_string(now->tm_min)+":"+to_string(now->tm_sec);
	current_status["timestamp"] = timestamp;
	current_status["direction"] = angle_converter(yaw);
	current_status["battery_level"] = battery_value;
	current_status["distance_to_dock"] = distance_to_docking;
	//current_status["RAM_in_use_sys"] = RAM_in_use;
	//current_status["RAM_free"] = freeRAM;
	
	Json::StyledWriter styledWriter;
	std::ofstream fid;
	fid.open(filename);
	fid  << styledWriter.write(current_status);
	fid.close();
}
