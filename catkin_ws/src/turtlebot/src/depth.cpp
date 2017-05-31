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
    //std::cout<<"in try"<<std::endl;
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
        /*uint16_t z_raw = depth_img.at<uint16_t>(320,240);
        float z_mean = z_raw*0.001;
        std::cout<<"point at 100, 100: "<<z_mean<<std::endl;*/
        /*cv::Mat mask = depth_img>0;
        double mmin = 0.0;
        double mmax = 0.0;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(depth_img, &mmin, &mmax, 0, 0, mask);
        std::cout<<"max value: "<<mmax<<". min value: "<<mmin<<std::endl;*/
  

        //show the original depth image after normalizing
        /*double max = 0.0;
        cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
        cv::Mat normalized;
        cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0);
        cv::imshow("foo", normalized);
        cv::waitKey(1);*/

        //corp the depth image
        cv::Mat corp_depth = depth_img(cv::Rect_<int>(180,150,280,310));
        cv::Mat mask = corp_depth>0;
        double mmin = 0.0;
        double mmax = 0.0;
        //cv::Point min_loc, max_loc;
        cv::minMaxLoc(corp_depth, &mmin, &mmax, 0, 0, mask);
        std::cout<<"max value: "<<mmax<<". min value: "<<mmin<<std::endl;
        int num = countNonZero(corp_depth);
        //std::cout<<"number of points with non-zero depth: "<<num<<"/86800"<<std::endl;
        float percent  = ((double)num)/129200.0;
        //std::cout<<"show the percentage: "<<percent<<std::endl;
        double max = 0.0;
        cv::minMaxLoc(corp_depth, 0, &max, 0, 0);
        //std::cout<<"what is the max: "<<max<<std::endl;
        cv::Mat corp_norm;
        corp_depth.convertTo(corp_norm, CV_32F, 1.0/max, 0);
        cv::imshow("foo", corp_norm);
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
