#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

void imgcb(const sensor_msgs::Image::ConstPtr& msg){
    //std::cout << "Top-left corner: " << *reinterpret_cast<const float*>(&msg->data[0]) << "m" << std::endl;
    //std::cout<<"image height: "<<msg->height<<std::endl; //480
    //std::cout<<"image width: "<<msg->width<<std::endl; //640
    //image width 640, image height 480
    try{
    std::cout<<"in try"<<std::endl;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);

        //
        cv::Mat depth_img;
        const std::string& enc = msg->encoding;
        if(enc.compare("16UC1") == 0)
            depth_img = cv_bridge::toCvShare(msg)->image;
        else if(enc.compare("32FC1") == 0)
            cv_ptr->image.convertTo(depth_img, CV_16UC1, 1000.0);
        //get raw distance value
        uint16_t z_raw = depth_img.at<uint16_t>(320,240);
        float z_mean = z_raw*0.001;
        std::cout<<"point at 100, 100: "<<z_mean<<std::endl;
        double mmin = 0.0;
        double mmax = 0.0;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(depth_img, &mmin, &mmax, &min_loc, &max_loc);
        std::cout<<"max value: "<<mmax<<". min value"<<mmin<<std::endl;
        int num = countNonZero(depth_img);
        std::cout<<"number of points with non-zero depth: "<<num<<"/307200"<<std::endl;
        /*cv::imshow("foo", depth_img);
        cv::waitKey(1);*/
        //

        double max = 0.0;
        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
        cv::Mat normalized;
        cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0);

        //float dist_val = cv_ptr->image.at<float>(100,100);
        //std::cout<<"point at 100,100: "<<dist_val<<std::endl;
        
        cv::imshow("foo", normalized);
        cv::waitKey(1);
    }catch (const cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "foo");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/depth/image", MY_ROS_QUEUE_SIZE, imgcb);

    cv::namedWindow("foo");
    ros::spin();
    cv::destroyWindow("foo");

    return 0;
}
