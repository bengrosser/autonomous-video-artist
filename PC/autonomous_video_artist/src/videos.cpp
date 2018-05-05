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
	time_t t = std::time(0);
	struct tm * now = localtime(&t);
	string timestamp = to_string(now->tm_year + 1900)+"-"+to_string(now->tm_mon+1)+"-"+to_string(now->tm_mday)+"_"+to_string(now->tm_hour)+":"+to_string(now->tm_min)+":"+to_string(now->tm_sec);


	if(shoot)
	{
		//string command = "ssh ubuntu@192.168.1.105 'gst-launch --eos-on-shutdown v4l2src num-buffers=600 device=\"/dev/video"+to_string(camera_idx)+"\" ! video/x-raw-yuv, width=640, height=480, framerate=30/1 ! ffmpegcolorspace ! jpegenc ! avimux ! filesink location=/media/ubuntu/6438-3534/clip"+to_string(clip_idx)+".mp4'";
		
		//write a json file before recording the video
		CorrespondingJson(timestamp, "start");

		string command = "ssh ubuntu@192.168.1.105 'gst-launch --eos-on-shutdown v4l2src num-buffers="+to_string(30*capture_duration)+"device=\"/dev/video"+to_string(camera_idx)+"\" ! video/x-raw-yuv, width=640, height=480, framerate=30/1 ! ffmpegcolorspace ! jpegenc ! avimux ! filesink location=/media/ubuntu/6438-3534/clips"+timestamp+".mp4'";
		system(command.c_str());
		++clip_idx;
		shoot = false;
		prev_shoot_timestamp = ros::Time::now();

		//write a json file after recording the video
		CorrespondingJson(timestamp, "end");
	}
}


void AutoNav::CorrespondingJson(string timestamp, string option)
{
	Json::Value current_status;
	
	struct sysinfo si;
	sysinfo(&si);
	current_status["RAM_in_use_sys"] = to_string(si.totalram-si.freeram);   //string type
	current_status["RAM_free"] = to_string(si.freeram);   //string type
	current_status["avg_brightness"] = brightness;
	current_status["avg_distance"] = avg_front_distance;
	current_status["battery_level"] = battery_value;
	current_status["capture_duration"] = capture_duration;
	current_status["capture_pan_type"] = panning_idx;
	current_status["capture_success"] = true;//TODO add this parameter
	current_status["direction"] = angle_converter(yaw);
	current_status["distance_to_dock"] = distance_to_docking;
	current_status["entropy"] = entropy;
	current_status["has_obstacle"] = !move_forward;
	current_status["motion_detected"] = motion;	
	current_status["position_X"] = current_x;
    current_status["position_Y"] = current_y;
	current_status["swap_free"]	= to_string(si.freeswap); //string type
	current_status["swap_in_use_sys"] = to_string(si.totalswap-si.freeswap);   //string type
	current_status["timestamp"] = timestamp;
	current_status["uptime"] = to_string(si.uptime); //string type
	

	string filename = "jsons"+timestamp+"-"+option;
	
	Json::StyledWriter styledWriter;
	std::ofstream fid;
	fid.open(filename);
	fid  << styledWriter.write(current_status);
	fid.close();
}
