/**********************
* A class for the autonomous video artist,
* This part focus on the preliminary analysis.
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/


#include "AutoNav.h"

vector<double> AutoNav::colorPercent(const cv_bridge::CvImageConstPtr cv_ptr, int group_num)
{
    vector<double> ret(pow(group_num, 3), 0.0);
    cv::Mat rgb_img = cv_ptr->image;
    int num_per_group = 255/group_num;
    for(int i = 0; i < img_height; ++i)
    {
        for(int j = 0; j < img_width; ++j)
        {
            int b = rgb_img.at<cv::Vec3b>(i,j)[0]/num_per_group;
            if(b == group_num)
                b = b-1;
            int g = rgb_img.at<cv::Vec3b>(i,j)[1]/num_per_group;
            if(g == group_num)
                g = g-1;
            int r = rgb_img.at<cv::Vec3b>(i,j)[2]/num_per_group;
            if(r == group_num)
                r = r-1;
            ret[r*pow(group_num, 2) + g*group_num + b]++;
        }
    }
    int total_pix = img_height*img_width;
    for(int i = 0; i < ret.size(); ++i)
    {
        ret[i] = ret[i]/total_pix;
    }
    return ret;
}

unsigned int AutoNav::count_bits(int n)
{
    int count = 0;
    while(n)
    {
        n &= n-1;
        count++;
    }
    return count;
}
bool AutoNav::bitAnalysis(const cv::Mat rgb_img)
{
    Mat gray_img;
    cvtColor(rgb_img, gray_img, COLOR_BGR2GRAY);
    int sum = 0;
    for(int i=0; i < gray_img.rows; ++i)
    {
        for(int j = 0; j < gray_img.cols; ++j)
        {
            int gray_intensity = gray_img.at<uint8_t> (i,j);
            sum += count_bits(gray_intensity);
        }
    }
    if(sum % 42 == 0)
    {
        printf("bit analysis is good\n");
        return true;
    }
    else
    {
        printf("bit analysis is bad\n");
        return false;
    }
}

void AutoNav::preAnalysis(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if(enc::isColor(msg->encoding))
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else
            cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        cv::Mat rgb_img = cv_ptr->image;
        cv::imshow("view", rgb_img);
        cv::waitKey(1);
        //bit = bitAnalysis(rgb_img);
        entropy = image_entropy(rgb_img);
        brightness = avg_brightness(rgb_img);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

double AutoNav::avg_distance(const cv::Mat depth_img)
{
    Mat mask = depth_img > 0;
    int pixel_num = countNonZero(mask);
    if(pixel_num==0)
        return -1;
    cv::Mat depth = depth_img/1000.0;
    return (double)sum(depth)[0]/(double)pixel_num;
}

double AutoNav::image_entropy(const cv::Mat rgb_img)
{
    Mat gray_img;
    if(rgb_img.channels() == 3)
        cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;

    cv::Mat hist;
    calcHist(&gray_img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
    hist /= gray_img.total();
    
    cv::Mat logP;
    cv::log(hist, logP);
    
    double entropy = -1*sum(hist.mul(logP)).val[0];
    return entropy;
}

double AutoNav::avg_brightness(const cv::Mat rgb_img)
{
    cv::Mat hsv_img;
    cvtColor(rgb_img, hsv_img, CV_BGR2HSV);
    vector<Mat> channel;
    split(hsv_img, channel);
    Scalar m = mean(channel[2]);
    return m[0];
}

bool AutoNav::motion_detection(const cv::Mat depth_img)
{
    Mat diff=cv::Mat::zeros(480,640,CV_32FC1);
    Mat diff_after = cv::Mat::zeros(480,640,CV_32FC1);
    
    if(first_time)
    {
        prev_frame = depth_img;
        first_time = false;
        return false;
    }
    else
    {
        Mat mask1 = prev_frame>0;
        Mat mask2 = depth_img > 0;
        Mat mask3;
        bitwise_and(mask1, mask2, mask3);
        cv::absdiff(depth_img, prev_frame, diff);
        mask1 = diff < 5000;
        bitwise_and(mask1, mask3, mask2);
        mask3 = diff > 1000;
        bitwise_and(mask2, mask3, mask1);
        Mat norm;
        mask1.convertTo(norm, CV_8U, 255.0);
        
        blur(norm, norm, Size(10,10));
        blur(norm, norm, Size(10,10));
        cv::Mat new_mask = norm>150;
        /*cv::imshow("norm", norm);
        cv::imshow("mask", new_mask);
        cv::waitKey(1);*/
        
        Mat new_mask1;
        new_mask.convertTo(new_mask1, CV_32FC1);
        
        motion_map = new_mask1+motion_map;
        
        //visualize
        double motion_max = 0.0;
        cv::minMaxLoc(motion_map, 0, &motion_max, 0, 0);
        Mat motion_norm;
        motion_map.convertTo(motion_norm, CV_32F, 1.0/motion_max, 0);
        cv::imshow("motion", motion_norm);
        cv::waitKey(1);


        Moments mu = moments(new_mask, true);
        Point center;
        center.x = mu.m10 / mu.m00;
        center.y = mu.m01 / mu.m00;
        Mat3b res;
        cvtColor(new_mask, res, CV_GRAY2BGR);
        prev_frame = depth_img;
        if(center.x > 0 && center.y > 0){
            circle(res, center, 20, Scalar(0,0,255), 4, 8, 0);
            imshow("result", res);
            cv::waitKey(1);
            return true;
        }
        else
            return false;

        
    }
}
