/**********************
* A class for the autonomous video artist,
* This part focus on the vision analysis.
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/

#include "AutoNav.h"

void AutoNav::frontEnv(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
        cv::Mat depth_img;
        const std::string& enc = msg->encoding;
        if(enc.compare("16UC1") == 0){
            depth_img = cv_ptr->image;
            printf("herhe\n");
        }
        else if(enc.compare("32FC1") == 0){
            cv_ptr->image.convertTo(depth_img, CV_16UC1, 1000.0);
            printf("really\n");
        }        

        avg_front_distance = avg_distance(depth_img);
        cv::Mat crop_front = depth_img(cv::Rect_<int>(180,200,280,270));
        cv::Mat mask = crop_front>0;
        int num = countNonZero(mask);
        float percentage = ((double)num)/((double)280*330);
        
        double mmin = 0.0;
        double mmax = 0.0;
        cv::minMaxLoc(crop_front, &mmin, &mmax, 0, 0, mask);
        if(mmin < 600 || percentage < 0.65)
        {
            cv::Mat crop_left = depth_img(cv::Rect_<int>(0,200,180,270));
            cv::Mat left_mask = crop_left > 0;
            cv::Mat crop_right = depth_img(cv::Rect_<int>(460,200,180,270));
            cv::Mat right_mask = crop_right > 0;
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

            if(battery_is_low && !near_docking_station)
            {
                cv::Mat front_right = depth_img(cv::Rect_ <int> (0, 200, 320, 270));
                cv::Mat front_left = depth_img(cv::Rect_ <int> (320, 200, 320, 270));
                cv::Mat front_right_mask = front_right > 650;
                cv::Mat front_left_mask = front_left > 650;
                if(countNonZero(front_right_mask) < countNonZero(front_left_mask))
                    avoid_from_right = true;
                else
                    avoid_from_right = false;
            }
            move_forward = false;
        }
        else
            move_forward = true;
        double max = 0.0;
        cv::minMaxLoc(crop_front, 0, &max, 0, 0);
        cv::Mat crop_norm;
        crop_front.convertTo(crop_norm, CV_32F, 1.0/max, 0);
        cv::imshow("foo", crop_norm);
        cv::waitKey(1);

    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
