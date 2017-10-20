/*******************
* A publisher to publish system information on the robot to a topic
*
*
* Created on 10/20/2017 
* Author: Xinyu Zhang 
* Contact: xzhan140@illinois.edu
*
********************/



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sys/sysinfo.h>

using namespace std;

/*
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;

    ros::Publisher sys_pub = n.advertise<std_msgs::Int32>("sys/freeRAM", 1000);
    ros::Rate loop_rate(10);
   
    while(ros::ok())
    {
        struct sysinfo si;
        sysinfo(&si);
        std_msgs::Int32 msg;
        msg.data = si.freeram;
        sys_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
*/

//using multiArray for sys information

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;
	
	ros::Publisher sys_pub1 = n.advertise<std_msgs::Int64>("sys/freeRAM", 100);
	ros::Publisher sys_pub2 = n.advertise<std_msgs::Int64>("sys/totalRAM", 100);
	ros::Publisher sys_pub3 = n.advertise<std_msgs::Int64>("sys/freeSwap", 100);
	ros::Publisher sys_pub4 = n.advertise<std_msgs::Int64>("sys/totalSwap", 100);
 	ros::Publisher sys_pub5 = n.advertise<std_msgs::Int64>("sys/uptime", 100);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		struct sysinfo si;
		sysinfo(&si);
		std_msgs::Int64 msg;
		msg.data = si.freeram;
		sys_pub1.publish(msg);

		msg.data = si.totalram;
		sys_pub2.publish(msg);

		msg.data = si.freeswap;
		sys_pub3.publish(msg);
	
		msg.data = si.totalswap;
		sys_pub4.publish(msg);
	
		msg.data = si.uptime;
		sys_pub5.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
