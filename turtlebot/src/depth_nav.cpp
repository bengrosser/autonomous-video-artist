#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <sys/sysinfo.h> //for system infomation
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h>

using namespace std;

static const uint8_t MAX_BATTERY = 162;
static const long minute = 60;
static const long hour = 3600; //minute*60
static const long day = 86400; //hour*24
static const double megabyte = 1024 * 1024;
static const double pi = 4*atan(1);   //pre-define pi

//for the angular velocity, positive means counterclockwise, negative means clockwise
class AutoNav
{
    private:
        ros::NodeHandle node;
        ros::Publisher velocity;
        bool move_forward;
        bool bump;
        int which_bumper;
        int img_height;  //image height was 480 by default
        int img_width;  //image width was 640 by default
        bool go_right;  
        bool battery_is_low;
        bool battery_is_full;   //when battery is charged to full, turtlebot should leave docking station
        bool in_charging;  //this boolean variable means the robot is charging
        bool near_docking_station;
        bool leave_docking_station;   
        double near_docking_station_x;
        double near_docking_station_y;
        double docking_station_x;
        double docking_station_y;


    public:
        //constructor
        AutoNav(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1)), move_forward(true), bump(false), img_height(480), img_width(640), go_right(false), battery_is_low(false), battery_is_full(true), near_docking_station(false), in_charging(false), leave_docking_station(true), near_docking_station_x(-0.782522), near_docking_station_y(0.077970), docking_station_x(0.0), docking_station_y(0.0){
            //make the robot move backward and turn 180 degree 
            /*geometry_msgs::Twist OUT_OF_DOCKING_STATION;
            OUT_OF_DOCKING_STATION.linear.x = -0.16;
            OUT_OF_DOCKING_STATION.angular.z = 0.0;
            ros::Time OUT_OF_DOCKING_TIME = ros::Time::now();
            while(ros::Time::now() - OUT_OF_DOCKING_TIME < ros::Duration(5.0))
                velocity.publish(OUT_OF_DOCKING_STATION);
            OUT_OF_DOCKING_STATION.linear.x = 0.0;
            OUT_OF_DOCKING_STATION.angular.z = 1.0;
            OUT_OF_DOCKING_TIME = ros::Time::now();
            while(ros::Time::now() - OUT_OF_DOCKING_TIME < ros::Duration(3.6))    //3.6
                velocity.publish(OUT_OF_DOCKING_STATION);*/            
            //Now the robot is ready to go
            
            ros::MultiThreadedSpinner threads(7);
            //create a thread for vision detection
            ros::Subscriber frontEnv=node.subscribe("/camera/depth/image", 1, &AutoNav::frontEnv, this);
            //create a thread to control the base 
            ros::Timer pilot=node.createTimer(ros::Duration(0.1), &AutoNav::pilot, this);
            //create a thread to detect bumper event
            ros::Subscriber bumperCommand=node.subscribe("/mobile_base/events/bumper",100, &AutoNav::bumperCommand, this);
            //create a thread to record the position
            ros::Subscriber position=node.subscribe("/odom", 1000, &AutoNav::position, this);
            //create a thread for battery information
            ros::Subscriber battery=node.subscribe("/mobile_base/sensors/core", 100, &AutoNav::battery, this);
            //create a thread for system information
            ros::Timer sysInfo=node.createTimer(ros::Duration(1), &AutoNav::sysInfo, this);
            //create a thread for automatic charging with docking station
            ros::Subscriber autoCharging=node.subscribe("/odom", 1000, &AutoNav::autoCharging, this);
            //the thread will loop until SIGINT (ctrl+c) is sent
            threads.spin();
        }

        void frontEnv(const sensor_msgs::Image::ConstPtr& msg){
            try{
                cv_bridge::CvImageConstPtr cv_ptr;
                cv_ptr = cv_bridge::toCvShare(msg);
                cv::Mat depth_img;
                const std::string& enc = msg->encoding;
                if(enc.compare("16UC1") == 0)
                    depth_img = cv_bridge::toCvShare(msg)->image;
                else if(enc.compare("32FC1") == 0)
                    cv_ptr->image.convertTo(depth_img, CV_16UC1, 1000.0);

                //crop the image
                cv::Mat crop_front = depth_img(cv::Rect_<int>(180,150,280,330)); //depth image in front

                cv::Mat mask = crop_front>0;  //mask to remove the noise
                int num = countNonZero(crop_front);
                float percentage = ((double)num)/((double)280*330); //change with the depth image size
               
                double mmin = 0.0;
                double mmax = 0.0;
                cv::minMaxLoc(crop_front, &mmin, &mmax, 0, 0, mask);
                //std::cout<<"max value: "<<mmax<<". min value: "<<mmin<<std::endl;
                if(mmin < 600 || percentage < 0.65){
                    //choose direction (BETA version)
                    cv::Mat crop_left = depth_img(cv::Rect_<int>(0,150,180,330)); //depth image on left
                    cv::Mat left_mask = crop_left>0;
                    cv::Mat crop_right = depth_img(cv::Rect_<int>(460,150,180,330)); //depth image on right
                    cv::Mat right_mask = crop_right>0;
                    double lmin = 0.0;
                    double lmax = 0.0;
                    double rmin = 0.0;
                    double rmax = 0.0;
                    cv::minMaxLoc(crop_left, &lmin, &lmax, 0, 0, left_mask);
                    cv::minMaxLoc(crop_right, &rmin, &rmax, 0, 0, right_mask);
                    if(lmin < rmin)
                        go_right = true;
                    else
                        go_right = false;
                    //
                    move_forward = false;
                }
                else
                    move_forward = true;

                //visualization
                double max = 0.0;
                cv::minMaxLoc(crop_front, 0 , &max, 0, 0);
                cv::Mat crop_norm;
                crop_front.convertTo(crop_norm, CV_32F, 1.0/max, 0);
                cv::imshow("foo", crop_norm);
                cv::waitKey(1);
                                
            }catch (const cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }

        void pilot(const ros::TimerEvent& time){
            //std::cout<<"pilot"<<std::endl;
            if(leave_docking_station){ //
                geometry_msgs::Twist OUT_OF_DOCKING_STATION;
                OUT_OF_DOCKING_STATION.linear.x = -0.16;
                OUT_OF_DOCKING_STATION.angular.z = 0.0;
                ros::Time OUT_OF_DOCKING_TIME = ros::Time::now();
                while(ros::Time::now() - OUT_OF_DOCKING_TIME < ros::Duration(5.0))
                    velocity.publish(OUT_OF_DOCKING_STATION);
                OUT_OF_DOCKING_STATION.linear.x = 0.0;
                OUT_OF_DOCKING_STATION.angular.z = 1.0;
                OUT_OF_DOCKING_TIME = ros::Time::now();
                while(ros::Time::now() - OUT_OF_DOCKING_TIME < ros::Duration(3.5))    //3.6
                    velocity.publish(OUT_OF_DOCKING_STATION);
                leave_docking_station = false;
            }
            else if(battery_is_low == false){
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
                        start = ros::Time::now();
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
                        if(DRIVE)
                            velocity.publish(decision);
                    }
                    else{
                        //BETA version
                        if(go_right)
                            decision.angular.z = DRIVE_ANGULARSPEED;
                        else
                            decision.angular.z = -DRIVE_ANGULARSPEED;

                        if(DRIVE){
                            while(!move_forward){
                                velocity.publish(decision);
                            }
                        }
                    } 
                }
            }
            else{//battery is low (navigate to the docking station)
                bool DRIVE;
                node.getParamCached("drive", DRIVE);       
                if(near_docking_station){ //start auto docking
                    if(!in_charging){
                        if(DRIVE){
                            actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> client("dock_drive_action", true);
                            client.waitForServer();
                            kobuki_msgs::AutoDockingGoal goal;
                            actionlib::SimpleClientGoalState dock_state = actionlib::SimpleClientGoalState::LOST;
                            client.sendGoal(goal);
                            ros::Time time = ros::Time::now();
                            while(!client.waitForResult(ros::Duration(3))){
                                dock_state = client.getState();
                                ROS_INFO("Docking status: %s", dock_state.toString().c_str());
                        
                                if(ros::Time::now() > (time+ros::Duration(500))){
                                    //Give it 500 seconds, or we say that auto docking fail.
                                    ROS_INFO("Docking took more than 500 seconds, canceling.");
                                    client.cancelGoal();
                                    break;
                                }
                            }
                            in_charging = true;
                        }
                    }
                }
                else{ //let the robot go to (near_docking_station_x, near_docking_station_y)
                    if(DRIVE){                   
                    
                    }
                }
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
            //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            //ROS_INFO("Orientation -> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        }

        void battery(const kobuki_msgs::SensorState msg){
            if(!in_charging){
                ros::Time start = ros::Time::now();
                while(ros::Time::now()-start < ros::Duration(5.0)){
                    //do nothing, just to waste the time
                }
                float percentage = ((float)msg.battery)/((float)MAX_BATTERY)*100.00;
                //ROS_INFO("left battery percentage %.2f %%", percentage);
                if(percentage < 0.3){
                    battery_is_low = true;
                    battery_is_full = false;
                }
                //else
                    //battery_is_low = false;
            }
            else{
                ros::Time start = ros::Time::now();
                if(msg.battery >= 159){
                    leave_docking_station = true;
                    battery_is_full = true;
                    in_charging = false;
                    battery_is_low = false;
                    near_docking_station = false;
                }
            }
        }

        void sysInfo(const ros::TimerEvent& time){
            //files in /proc are about system info like "/proc/meminfo" 
            struct sysinfo si;
            sysinfo(&si);
            /*printf ("system uptime : %ld days, %ld:%02ld:%02ld\n", si.uptime / day, (si.uptime % day) / hour, (si.uptime % hour) / minute, si.uptime % minute);
            printf ("total RAM   : %5.1f MB\n", si.totalram / megabyte);
            printf ("free RAM   : %5.1f MB\n", si.freeram / megabyte);
            printf ("process count : %d\n", si.procs);*/
        }
        
        void autoCharging(const nav_msgs::Odometry::ConstPtr& msg){
            //std::cout<<"test test"<<std::endl;
            float distance_to_docking = sqrt(pow(msg->pose.pose.position.x, 2.0)+pow(msg->pose.pose.position.y, 2.0));
            if(distance_to_docking < 1.5)
                near_docking_station = true;
            //else
            //    near_docking_station = false;
            float roll;
            float pitch;
            float yaw;
            float x = msg->pose.pose.orientation.x;
            float y = msg->pose.pose.orientation.y;
            float z = msg->pose.pose.orientation.z;
            float w = msg->pose.pose.orientation.w;
            toEulerianAngle(x,y,z,w,roll, pitch, yaw);
            //std::cout<<"roll: "<<roll<<std::endl;
            //std::cout<<"pitch: "<<pitch<<std::endl;
            //std::cout<<"yaw: "<<yaw<<std::endl;
        }

        //a helper function (convert quaternion angle to euler angle)
        void toEulerianAngle(const float x, const float y, const float z, const float w, float& roll, float& pitch, float& yaw){
            float ysqr = y*y;
            //roll (x-axis rotation)
            float t0 = +2.0*(w*x+y*z);
            float t1 = +1.0-2.0*(x*x+ysqr);
            roll = std::atan2(t0, t1);

            //pitch (y-axis rotation)
            float t2 = +2.0*(w*y-z*x);
            t2 = t2>1.0 ? 1.0 : t2;
            t2 = t2<-1.0 ? -1.0 : t2;
            pitch = std::asin(t2);

            //yaw (z-axis rotation)
            float t3 = +2.0*(w*z+x*y);
            float t4 = +1.0-2.0*(ysqr+z*z);
            yaw = std::atan2(t3, t4);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation");
    ros::NodeHandle node("autoNav");

    //initial parameter value
    node.setParam("drive_linearspeed",0.07); //Set the linear speed for the turtlebot
    node.setParam("drive_angularspeed",0.18);  //Set the angular spped
    node.setParam("drive",true); //For debugging, always set to true

    AutoNav turtlebot(node); 

    //clean up
    node.deleteParam("drive_linearspeed");
    node.deleteParam("drive_angularspeed");
    node.deleteParam("drive");
    return 0;
}
