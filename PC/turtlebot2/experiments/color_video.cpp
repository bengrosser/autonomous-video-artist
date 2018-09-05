#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

VideoWriter video;

/*void take_video(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if(enc::isColor(msg->encoding))
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else
            cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        cv::Mat rgb_img = cv_ptr->image;
        cv::imshow("rgb", rgb_img);
        cv::waitKey(1);
        video.write(rgb_img);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


int main(int argc, char ** argv)
{
    video.open("video1.avi", CV_FOURCC('M','J','P','G'), 1, Size(640,480), true);
    if(!video.isOpened()){
        printf("Not able to open file!!\n");
        return 0;
    }
    ros::init(argc, argv, "color");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("/camera/rgb/image_raw", 10, take_video);
    ros::spin();
    video.release();
    return 0;
}*/


void videoCapture(double duration, int camera_idx, int video_idx){
    cout<<"camera"<<camera_idx<<endl;
    VideoCapture vcap(camera_idx);
    if(!vcap.isOpened()){
        cout<<"Erros openning video stream or file"<<endl;
        return;
    }
    int frame_width = vcap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
    string output_file = "camera"+to_string(camera_idx)+"output"+to_string(video_idx)+".avi";
    VideoWriter video(output_file, CV_FOURCC('M','J','P','G'),22, Size(frame_width, frame_height), true);
   
    time_t start = time(NULL);
    while(difftime(time(NULL), start) < duration ){
        double elapsed = difftime(time(NULL), start);
        //std::cout<<"How long is the time elapsed: "<<elapsed<<std::endl;
        Mat frame;
        vcap >> frame;
        video.write(frame);
        imshow("Frame", frame);
        waitKey(1);
    }
    vcap.release();
    video.release();
    ++video_idx;
}

int main(){
    videoCapture(10, 0, 1);
    return 0;;
}
