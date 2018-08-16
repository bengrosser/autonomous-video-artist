#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <Transform.h>
#include <transform_datatypes.h>


using namespace std;


class AutoNav
{
	private:
		ros::NodeHandle node;
		ros::Publisher velocity;
		bool move_forward;
		float LIDAR_ERR;
		
	public:
		AutoNav(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("cmd_vel", 10), move_forward(true), LIDAR_ERR(0.05))
		{
			ros::MultiThreadedSpinner threads(2);
			
			ros::Subscriber scan = node.subscribe<sensor_msgs::LaserScan>("/scan", 10, &AutoNav::LaserInfo, this);
			ros::Timer pilot = node.createTimer(ros::Duration(0.1), &AutoNav::pilot, this);
			threads.spin();
		}

		void LaserInfo(const sensor_msgs::LaserScan::ConstPtr& scan)
		{
			vector<float> scan_filter;
			for(int i = 0; i <= 20; ++i){
				if(scan->ranges[i] >= LIDAR_ERR)
					scan_filter.push_back(scan->ranges[i]);
			}
			for(int i = 340; i < 360; ++i){
				if(scan->ranges[i] >= LIDAR_ERR)
					scan_filter.push_back(scan->ranges[i]);
			}
			if(scan_filter.size() == 0){
				move_forward = false;
			}
			else{
				vector<float>::iterator it = min_element(scan_filter.begin(), scan_filter.end());
				if(*it < 0.3){
					move_forward = false;
				}
				else{
					move_forward = true;
				}
			}
			if(move_forward = true){
				geometry_msgs::Twist decision;
				decision.linear.x = 0.05;
				decision.angular.z = 0.0;
				velocity.publish(decision);
			}
			else{
				geometry_msgs::Twist decision;
				decision.linear.x = 0.0;
				decision.angular.z = 0.0;
				velocity.publish(decision);
				int random = rand()%2;
				if(random == 0){
					int prev = 20;
					for(int i = 21; i < 360; ++i){
						if(i-prev > 40)
							break;
						if(scan->ranges[i] > LIDAR_ERR && scan->ranges[i] < 0.3)
							prev = i;
					}
					
				}
				else{

				}
			}
		}

		void pilot(const ros::TimerEvent& time)
		{
			
		}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation");
	ros::NodeHandle node("navigation");
	
	
	AutoNav turtlebot(node);

	
	return 0;
}
