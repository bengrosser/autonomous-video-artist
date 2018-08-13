#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

using namespace std;

void LaserInfo(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float t = scan->ranges[0];
	cout<<t<<endl;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot3");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, LaserInfo);
	ros::spin();
}
