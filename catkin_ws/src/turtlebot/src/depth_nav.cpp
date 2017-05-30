#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

class AutoNav
{
    private:
        ros::NodeHandle node;
        ros::Publisher velocity;
        bool move_forward;
        bool bump;
        int which_bumper;
        int img_height;
        int img_width;
        int corp_height; //height of corp_front
        int corp_width; //width of corp_front
        

    public:
        //constructor
        AutoNav(ros::NodeHandle& handle):node(handle), velocity(node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1)), move_forward(false), bump(false), img_height(480), img_width(640), corp_height(310), corp_width(280){
            ros::MultiThreadedSpinner threads(3);
            //create a thread for vision detection
            ros::Subscriber frontEnv=node.subscribe("/camera/depth/image", 1, &AutoNav::frontEnv, this);
            //create a thread to control the base 
            ros::Timer pilot=node.createTimer(ros::Duration(0.1), &AutoNav::pilot, this);
            //create a thread to detect bumper event
            ros::Subscriber bumperCommand=node.subscribe("/mobile_base/events/bumper",100, &AutoNav::bumperCommand, this);
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

                //corp the image  (need some test)
                cv::Mat corp_front = depth_img(cv::Rect_<int>(180,150,corp_width,corp_height)); //only for the image in front
                int num = countNonZero(corp_front);
                float percentage = ((double)num)/((double)corp_width*corp_height);
                if(percentage < 0.65)  //the threshold could be modified
                    move_forward = false;
                else
                    move_forward = true;

                //visualization
                double max = 0.0;
                cv::minMaxLoc(corp_front, 0 , &max, 0, 0);
                cv::Mat corp_norm;
                corp_depth.convertTo(corp_norm, CV_32F, 1.0/max, 0);
                cv::imshow("foo", corp_norm);
                cv::waitKey(1);
                                
            }catch (const cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

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
                std::cout<<"bump true"<<std::endl;
                which_bumper = msg.bumper;
                //sleep(5);
            }
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
