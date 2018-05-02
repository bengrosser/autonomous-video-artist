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


void AutoNav::shoot_video(const ros::TimerEvent& time)
{
	/*int hour = 0;
	int minute = 0;
	int second = 20;
	double speed = 1.0;
	int camera_idx = 0;
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
	CorrespondingJson(json_filename);*/

	time_t t = std::time(0);
	struct tm * now = localtime(&t);
	string timestamp = to_string(now->tm_year + 1900)+"-"+to_string(now->tm_mon+1)+"-"+to_string(now->tm_mday)+"_"+to_string(now->tm_hour)+":"+to_string(now->tm_min)+":"+to_string(now->tm_sec);


    //TODO write a json file before record the video
	string json_filename = timestamp+"_start.json";
	CorrespondingJson(json_filename);


	if(shoot)
	{
		//string command = "ssh ubuntu@192.168.1.105 'gst-launch --eos-on-shutdown v4l2src num-buffers=600 device=\"/dev/video"+to_string(camera_idx)+"\" ! video/x-raw-yuv, width=640, height=480, framerate=30/1 ! ffmpegcolorspace ! jpegenc ! avimux ! filesink location=/media/ubuntu/6438-3534/clip"+to_string(clip_idx)+".mp4'";
		string command = "ssh ubuntu@192.168.1.105 'gst-launch --eos-on-shutdown v4l2src num-buffers=600 device=\"/dev/video"+to_string(camera_idx)+"\" ! video/x-raw-yuv, width=640, height=480, framerate=30/1 ! ffmpegcolorspace ! jpegenc ! avimux ! filesink location=/media/ubuntu/6438-3534/"+timestamp+".mp4'";
		system(command.c_str());
		++clip_idx;
		shoot = false;
		prev_shoot_timestamp = ros::Time::now();
	}

	//TODO write a json file after recording the video
	json_filename = timestamp+"_end.json";
	CorrespondingJson(json_filename);
	
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

/*void AutoNav::CorrespondingJson(Json::Value& current_status)
{
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
	//TODO: long to int
	//current_status["RAM_in_use_sys"] = si.totalram-si.freeram;
	//current_status["RAM_free"] = si.freeram;
	//current_status["swap_in_use_sys"] = si.totalswap-si.freeswap;
	//current_status["swap_free"] = si.freeswap;
	//current_status["uptime"] = si.uptime;
	//current_status["capture_pan_type"]
	//current_status["capture_duration"]
	//current_status["capture_success"]
}*/
