#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>

using namespace std;

static const uint8_t MAX_BATTERY = 162;

class AutoNav
{
    private:
        ros::NodeHandle node;
        ros::Publisher velocity;
        ros::Publisher panorama;  //whole point cloud
        ros::Publisher front;  //point cloud at front
        bool move_forward;
        bool bump;
        int which_bumper;
        

    public:
        //constructor
        AutoNav(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1)), panorama(node.advertise<pcl::PointCloud<pcl::PointXYZ>>("panorama",1)), front(node.advertise<pcl::PointCloud<pcl::PointXYZ>>("front",1)), move_forward(false), bump(false){
            ros::MultiThreadedSpinner threads(5);
            //create a thread for vision detection
            ros::Subscriber frontEnv=node.subscribe("/camera/depth/points", 1, &AutoNav::frontEnv, this);
            //create a thread to control the base 
            ros::Timer pilot=node.createTimer(ros::Duration(0.1), &AutoNav::pilot, this);
            //create a thread to detect bumper event
            ros::Subscriber bumperCommand=node.subscribe("/mobile_base/events/bumper",100, &AutoNav::bumperCommand, this);
            //create a thread to record the position
            ros::Subscriber position=node.subscribe("/odom", 1000, &AutoNav::position, this);
            //create a thread for battery information
            ros::Subscriber battery=node.subscribe("/mobile_base/sensors/core", 100, &AutoNav::battery, this);
            //the thread will loop until SIGINT (ctrl+c) is sent
            threads.spin();
        }

        void frontEnv(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
            //std::cout<<"frontEnv"<<std::endl;
            double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX, DOWNSAMPLING;
            int SAMPLE_NUM;

            node.getParamCached("crop_xradius", CROP_XRADIUS);
            node.getParamCached("crop_ymin", CROP_YMIN);
            node.getParamCached("crop_ymax", CROP_YMAX);
            node.getParamCached("crop_zmin", CROP_ZMIN);
            node.getParamCached("crop_zmax", CROP_ZMAX);
            node.getParamCached("height_downsampling", DOWNSAMPLING);
            node.getParamCached("sample_num", SAMPLE_NUM);

            pcl::PassThrough<pcl::PointXYZ> crop;
            pcl::VoxelGrid<pcl::PointXYZ> downsample;
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr frontView(new pcl::PointCloud<pcl::PointXYZ>);

            downsample.setInputCloud(cloud);
            if(DOWNSAMPLING >= 0)
                downsample.setLeafSize((float)DOWNSAMPLING, (float)DOWNSAMPLING, (float)DOWNSAMPLING);
            downsample.filter(*downsampled);

            //crop the point cloud
            crop.setInputCloud(downsampled);
            crop.setFilterFieldName("x");
            crop.setFilterLimits(-CROP_XRADIUS, CROP_XRADIUS);
            crop.filter(*frontView);

            crop.setInputCloud(frontView);
            crop.setFilterFieldName("y");
            crop.setFilterLimits(CROP_YMIN, CROP_YMAX);
            crop.filter(*frontView);

            crop.setInputCloud(frontView);
            crop.setFilterFieldName("z");
            crop.setFilterLimits(CROP_ZMIN, CROP_ZMAX);
            crop.filter(*frontView);

            //if(!move_forward)
              //  frontSamples.clear();
            //std::cout<<"frontView size:"<<frontView->size()<<std::endl;
            /*frontSamples.push_front(frontView->size());
            while(frontSamples.size() > (unsigned) SAMPLE_NUM)
                frontSamples.pop_back();
            
            int averageObstacles = 0;
            for(std::list<int>::iterator location=frontSamples.begin(); location!=frontSamples.end(); ++location)
                averageObstacles+=(*location);

            averageObstacles /= frontSamples.size();
            std::cout<<averageObstacles<<std::endl;
            if(averageObstacles>0){*/
            if(frontView->size()>0){
                move_forward = false;
            }
            else
                move_forward = true;
            panorama.publish(*downsampled);
            front.publish(*frontView);
        }

        void pilot(const ros::TimerEvent& time){
            double DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
            bool DRIVE;
            node.getParamCached("drive_linearspeed", DRIVE_LINEARSPEED);
            node.getParamCached("drive_angularspeed", DRIVE_ANGULARSPEED);
            node.getParamCached("drive", DRIVE);

            geometry_msgs::Twist decision;
            if(bump){//deal with the bumper info
                if(DRIVE){
                    decision.linear.x = -DRIVE_LINEARSPEED;
                    decision.angular.z = 0;
                    ros::Time start = ros::Time::now();
                    while(ros::Time::now() - start < ros::Duration(5.0)){
                        velocity.publish(decision);
                    }
                    start = ros::Time::now();
                    //choose right and left by bumper
                    int direction = 1;
                    if(which_bumper == 1){
                        int tmp = rand()%2;
                        if(tmp == 0)
                            direction = 1;
                        else
                            direction = -1;
                    }
                    else if(which_bumper == 0){
                        direction = -1;
                    }
                    else{
                        direction = 1;
                    }
                    decision.angular.z = DRIVE_ANGULARSPEED*direction;
                    decision.linear.x = 0;
                    while(ros::Time::now() - start < ros::Duration(3.0)){
                        velocity.publish(decision);
                    }
                }
                bump = false;
            }
            else{//bumper is safe
                if(move_forward){
                    decision.linear.x=DRIVE_LINEARSPEED;
                    decision.angular.z = 0;
                }
                else{
                    int direction = rand()%2;
                    decision.angular.z = DRIVE_ANGULARSPEED;
                }
                if(DRIVE)
                    velocity.publish(decision);
            }
        }

        void bumperCommand(const kobuki_msgs::BumperEvent msg){
            if(msg.state){
                bump = true;
                which_bumper = msg.bumper;
            }
        }

        void position(const nav_msgs::Odometry::ConstPtr& msg){
            ros::Time start = ros::Time::now();
            while(ros::Time::now()-start < ros::Duration(5.0)){
                //do nothing, just to waste the time
            }
            ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        }

        void battery(const kobuki_msgs::SensorState msg){
            ros::Time start = ros::Time::now();
            while(ros::Time::now()-start < ros::Duration(5.0)){
                //do nothing, just to waste the time
            }
            float percentage = ((float)msg.battery)/((float)MAX_BATTERY)*100.00;
            ROS_INFO("left battery percentage %.2f %%", percentage);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation");
    ros::NodeHandle node("autoNav");

    //initial parameter value
    node.setParam("crop_xradius", 1); //slightly larger than the robot's radius
    node.setParam("crop_ymin", -0.5); //the absolute value should be slightly larger than the robot's height
    node.setParam("crop_ymax", 0.5); //slightly 
    node.setParam("crop_zmin", -1.0); //set it to zero
    node.setParam("crop_zmax", 1.5); //the distance that the robot start to avoid the obstacle
    node.setParam("downsampling", 0.04); //should be low enough to eliminate the noise
    node.setParam("sample_num", 5); //threshold to detect whether there is an obstacle at front
    node.setParam("drive_linearspeed",0.07); //Set the linear speed for the turtlebot
    node.setParam("drive_angularspeed",0.18);  //Set the angular spped
    node.setParam("drive",true); //For debugging, always set to true

    AutoNav turtlebot(node); 

    //clean up
    node.deleteParam("crop_xradius");
    node.deleteParam("crop_ymin");
    node.deleteParam("crop_ymax");
    node.deleteParam("crop_zmin");
    node.deleteParam("crop_zmax");
    node.deleteParam("downsampling");
    node.deleteParam("sample_num");
    node.deleteParam("drive_linearspeed");
    node.deleteParam("drive_angularspeed");
    node.deleteParam("drive");
    return 0;
}
