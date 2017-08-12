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

vector<double> AutoNav::colorPercent(cv_bridge::CvImageConstPtr cv_ptr, int group_num)
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

void AutoNav::bitAnalysis(const sensor_msgs::ImageConstPtr& msg)
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
        cv::Mat gray_img;
        cv::cvtColor(rgb_img, gray_img, COLOR_BGR2GRAY);
        int sum = 0;
        for(int i = 0; i < gray_img.rows; ++i)
        {
            for(int j = 0; j < gray_img.cols; ++j)
            {
                int gray_intensity = gray_img.at<uint8_t> (i,j);
                sum += count_bits(gray_intensity);
            }
        }
        if(sum % 42 == 0)
        {
            printf("it is good\n");
        }
        else
        {
            printf("it is bad\n");
        }
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
