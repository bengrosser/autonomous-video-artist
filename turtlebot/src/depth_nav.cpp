#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
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
        bool move_forward;
        bool bump;
        int which_bumper;
        int img_height;  //image height was 480 by default
        int img_width;  //image width was 640 by default
        bool go_right;
        

    public:
        //constructor
        AutoNav(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1)), move_forward(true), bump(false), img_height(480), img_width(640), go_right(false){
            ros::MultiThreadedSpinner threads(5);
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

                //corp the image
                cv::Mat corp_front = depth_img(cv::Rect_<int>(180,150,280,330)); //depth image in front

                cv::Mat mask = corp_front>0;  //mask to remove the noise
                int num = countNonZero(corp_front);
                float percentage = ((double)num)/((double)280*330); //change with the depth image size
                std::cout<<"percentage: "<<percentage<<std::endl;
               
                double mmin = 0.0;
                double mmax = 0.0;
                cv::minMaxLoc(corp_front, &mmin, &mmax, 0, 0, mask);
                std::cout<<"max value: "<<mmax<<". min value: "<<mmin<<std::endl;
                if(mmin < 600 || percentage < 0.65){
                    //choose direction (BETA version)
                    cv::Mat corp_left = depth_img(cv::Rect_<int>(0,150,180,330)); //depth image on left
                    cv::Mat left_mask = corp_left>0;
                    cv::Mat corp_right = depth_img(cv::Rect_<int>(460,150,180,330)); //depth image on right
                    cv::Mat right_mask = corp_right>0;
                    double lmin = 0.0;
                    double lmax = 0.0;
                    double rmin = 0.0;
                    double rmax = 0.0;
                    cv::minMaxLoc(corp_left, &lmin, &lmax, 0, 0, left_mask);
                    cv::minMaxLoc(corp_right, &rmin, &rmax, 0, 0, right_mask);
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
                cv::minMaxLoc(corp_front, 0 , &max, 0, 0);
                cv::Mat corp_norm;
                corp_front.convertTo(corp_norm, CV_32F, 1.0/max, 0);
                cv::imshow("foo", corp_norm);
                cv::waitKey(1);
                                
            }catch (const cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }

        void pilot(const ros::TimerEvent& time){
            //std::cout<<"pilot"<<std::endl;
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

                    //
                    //decision.angular.z = DRIVE_ANGULARSPEED; 
                    if(DRIVE){
                        while(!move_forward){
                            velocity.publish(decision);
                        }           
                    }
                }
               //if(DRIVE)
                    //velocity.publish(decision);
            }
        }

        void bumperCommand(const kobuki_msgs::BumperEvent msg){
            if(msg.state){
                bump = true;
                which_bumper = msg.bumper;
                //sleep(5);
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
