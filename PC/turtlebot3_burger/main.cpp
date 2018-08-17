#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <signal.h>


using namespace std;

static const double pi = 4*atan(1);

ros::Publisher velocity;

bool stop;

void SigintHandler(int sig){
	cout<<"signal handler"<<endl;
	stop = true;
	geometry_msgs::Twist decision;
	decision.linear.x = 0.0;
	decision.angular.z = 0.0;
	velocity.publish(decision);
	cout<<"set velocity to zero"<<endl;
	ros::shutdown();
}

class AutoNav
{
	private:
		ros::NodeHandle node;
		
		bool move_forward;
		float LIDAR_ERR;

	public:
		AutoNav(ros::NodeHandle& handle):node(handle), move_forward(true), LIDAR_ERR(0.05)
		{
			stop = false;
			velocity = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
			signal(SIGINT, SigintHandler);
			//ros::MultiThreadedSpinner threads(2);

			ros::Subscriber scan = node.subscribe<sensor_msgs::LaserScan>("/scan", 10, &AutoNav::LaserInfo, this);
			//ros::Timer pilot = node.createTimer(ros::Duration(0.1), &AutoNav::pilot, this);
			//threads.spin();
			ros::spin();
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
				cout<<"min element: "<<*it<<endl;
				if(*it < 0.3){
					move_forward = false;
				}
				else{
					move_forward = true;
				}
			}
			cout<<"move_forward: "<<move_forward<<endl;
			if(move_forward == true){
				geometry_msgs::Twist decision;
				if(!stop){
					decision.linear.x = 0.1;
					decision.angular.z = 0.0;
					velocity.publish(decision);
					cout<<"go straight"<<endl;
				}else{
					decision.linear.x = 0.0;
					decision.angular.z = 0.0;
					velocity.publish(decision);
				}
				//cout<<"go straight"<<endl;
			}
			else{
				/*geometry_msgs::Twist decision;
				decision.linear.x = 0.0;
				decision.angular.z = 0.1;
				velocity.publish(decision);
				tf::StampedTransform transform;
				tf::Quaternion q = transform.getRotation();
				double yaw = tf::getYaw(q);*/

				geometry_msgs::Twist decision;
				int random = rand()%2;
				if(random == 0){
					/*int prev = 20;
					for(int i = 21; i < 360; ++i){
						if(i-prev > 40)
							break;
						if(scan->ranges[i] > LIDAR_ERR && scan->ranges[i] < 0.3)
							prev = i;
					}
					double target = ((float)prev+20.0)/180.0*pi;
					while(yaw < target)*/
					decision.linear.x = 0.0;
					decision.angular.z = 0.2;
				}
				else{
					decision.linear.x = 0.0;
					decision.angular.z = -0.2;
				}
				ros::Time current_time = ros::Time::now();
				while(ros::Time::now()-current_time <= ros::Duration(3.0) && !stop){
					cout<<"in the while loop"<<endl;
					velocity.publish(decision);
					cout<<"turn around"<<endl;
				}
				if(stop){
					decision.linear.x = 0.0;
					decision.angular.z = 0.0;
					velocity.publish(decision);
				}
				move_forward = true;
			}
		}

		/*void pilot(const ros::TimerEvent& time)
		{

		}*/
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation");
	ros::NodeHandle node("navigation");


	AutoNav turtlebot(node);


	return 0;
}
