/**********************
* A class for the autonomous video artist,
* This part focus on the navigation functions
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/


//for the angular velocity, positive means counterclockwise, negative means clockwise
//for the bumper, 0: on the left, 1: in the middle, 2: on the right 

#include "AutoNav.h"

const double pi = 4*atan(1);

/*Function to calculate the speed during accelerating*/
float AutoNav::acc_speed(double target_velocity, double duration, double time_elapsed)
{
    return (target_velocity/(duration*duration))*time_elapsed*time_elapsed;
}


/*Function to calculate the speed during decelerating*/
float AutoNav::dec_speed(double start_velocity, double duration, double time_elapsed)
{
    return (-start_velocity/(duration*duration))*time_elapsed*time_elapsed+start_velocity;
}

void AutoNav::linear_accelerate(const ros::TimerEvent& time, double target_velocity, double duration)
{
    geometry_msgs::Twist decision;
    decision.linear.x = 0.0;
    decision.angular.z = 0.0;
    ros::Time current_time = ros::Time::now();
    while(ros::Time::now() - current_time <= ros::Duration(duration) && move_forward)
    {
        double time_elapsed = (ros::Time::now()-current_time).toSec();
        decision.linear.x = acc_speed(target_velocity, duration, time_elapsed);
        velocity.publish(decision);
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void AutoNav::angular_accelerate(const ros::TimerEvent& time, double target_velocity, double duration)
{
    geometry_msgs::Twist decision;
    decision.linear.x = 0.0;
    decision.angular.z = 0.0;
    ros::Time current_time = ros::Time::now();
    while(ros::Time::now() - current_time <= ros::Duration(duration))
    {
        double time_elapsed = (ros::Time::now()-current_time).toSec();
        decision.angular.z = acc_speed(target_velocity, duration, time_elapsed);
        velocity.publish(decision);
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void AutoNav::linear_decelerate(const ros::TimerEvent& time, double start_velocity, double duration)
{
    geometry_msgs::Twist decision;
    decision.linear.x = 0.0;
    decision.angular.z = 0.0;
    ros::Time current_time = ros::Time::now();
    while(ros::Time::now() - current_time <= ros::Duration(duration))
    {
        double time_elapsed = (ros::Time::now()-current_time).toSec();
        decision.linear.x = dec_speed(start_velocity, duration, time_elapsed);
        velocity.publish(decision);
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void AutoNav::angular_decelerate(const ros::TimerEvent& time, double start_velocity, double duration)
{
    geometry_msgs::Twist decision;
    decision.linear.x = 0.0;
    decision.angular.z = 0.0;
    ros::Time current_time = ros::Time::now();
    while(ros::Time::now() - current_time <= ros::Duration(duration))
    {
        double time_elapsed = (ros::Time::now()-current_time).toSec();
        decision.angular.z = dec_speed(start_velocity, duration, time_elapsed);
        velocity.publish(decision);
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void AutoNav::leave_station_action(const ros::TimerEvent& time)
{
    bool DRIVE;
    node.getParamCached("drive", DRIVE);
    if(DRIVE)
    {
        linear_accelerate(time, -0.16, 6.0);
        geometry_msgs::Twist decision;
        decision.linear.x = -0.16;
        decision.angular.z = 0.0;
        ros::Time start = ros::Time::now();
        while(ros::Time::now() -  start < ros::Duration(2.0))
        {
            velocity.publish(decision);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        decision.linear.x = 0.0;
        decision.angular.z = 1.0;
        start = ros::Time::now();
        while(ros::Time::now() - start < ros::Duration(3.6))
        {
            velocity.publish(decision);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
        leave_docking_station = false;
    }
}

void AutoNav::battery_is_good_action(const ros::TimerEvent& time)
{
    geometry_msgs::Twist decision;
    if(bump)
    {
        if(DRIVE)
        {
            linear_accelerate(time, -linear_speed, 5.0);
            decision.linear.x = -linear_speed;
            decision.angular.z = 0.0;
            ros::Time start = ros::Time::now();
            while(ros::Time::now() - start < ros::Duration(2.0))
            {
                velocity.publish(decision);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            int direction = 1;
            if(which_bumper == 1)
            {
                int tmp = rand()%2;
                if(tmp == 0)
                    direction = 1;
                else
                    direction = -1;
            }
            else if(which_bumper == 0)
            {
                direction = -1;
            }
            else
            {
                direction = 1;
            }
            decision.angular.z = direction*angular_speed;
            decision.linear.x = 0.0;
            start = ros::Time::now();
            while(ros::Time::now() - start < ros::Duration(3.0))
            {
                velocity.publish(decision);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        bump = false;
    }
    else
    {
        if(acc_or_not)
        {
            linear_accelerate(time, linear_speed, 5.0);
            acc_or_not = false;
        }
        if(move_forward)
        {
            decision.linear.x = linear_speed;
            decision.angular.z = 0.0;
            if(DRIVE)
            {
                velocity.publish(decision);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        else
        {
            if(go_right)
                decision.angular.z = angular_speed;
            else
                decision.angular.z = -angular_speed;

            if(DRIVE)
            {
                while(!move_forward)
                {
                    velocity.publish(decision);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            acc_or_not = true;
        }
    }
}

void AutoNav::auto_docking_action(const ros::TimerEvent& time)
{
    if(!in_charging)
    {
        actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> client("dock_drive_action", true);
        client.waitForServer();
        kobuki_msgs::AutoDockingGoal goal;
        actionlib::SimpleClientGoalState dock_state = actionlib::SimpleClientGoalState::LOST;
        client.sendGoal(goal);
        ros::Time time = ros::Time::now();
        while(!client.waitForResult(ros::Duration(3)))
        {
            dock_state = client.getState();
            ROS_INFO("Docking status: %s", dock_state.toString().c_str());
            in_charging = true;
            if(ros::Time::now() > (time+ros::Duration(50)))
            {
                ROS_INFO("Docking took more than 50 seconds, canceling.");
                client.cancelGoal();
                in_charging = false;
                break;
            }
        }
    }
}

void AutoNav::battery_is_low_action(const ros::TimerEvent& time)
{
    geometry_msgs::Twist decision;
    while(!near_docking_station)
    {
        float angle;
        if(current_y >= 0)
            angle = atan2(current_y, current_x-near_docking_station_x)-pi;
        else
            angle = atan2(current_y, current_x-near_docking_station_x)+pi;
        if(angle < -pi+0.02)
            angle = pi;
        decision.linear.x = 0;
        decision.angular.z = 0.4;
        while(abs(yaw-angle) > 0.02)
        {
            velocity.publish(decision);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        decision.linear.x = 0.2;
        decision.angular.z = 0;
        if(!bump)
        {
            ros::Time start = ros::Time::now();
            decision.linear.x = 0.15;
            decision.angular.z = 0.0;
            while(move_forward && !near_docking_station && !bump)
            {
                velocity.publish(decision);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if(ros::Time::now()-start >= ros::Duration(10.0))
                    break;
            }
            if(!move_forward && !near_docking_station && !bump)
            {
                if(avoid_from_right)
                    decision.angular.z = -0.15;
                else
                    decision.angular.z = 0.15;
                decision.linear.x = 0;
                while(!move_forward)
                {
                    velocity.publish(decision);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                decision.linear.x = 0.15;
                decision.angular.z = 0.0;
                start = ros::Time::now();
                while(ros::Time::now() - start < ros::Duration(4.0))
                {
                    velocity.publish(decision);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }
        else
        {
            decision.linear.x = -0.2;
            decision.angular.z = 0;
            ros::Time start = ros::Time::now();
            while(ros::Time::now()-start < ros::Duration(2.5))
            {
                velocity.publish(decision);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            float cur_yaw = yaw;
            float target_yaw;
            if(which_bumper == 0)
            {
                target_yaw = cur_yaw-pi/4 < (-pi) ? (2*pi+cur_yaw-pi/4) : (cur_yaw - pi/4);
                if(target_yaw < -pi+0.02)
                    target_yaw = pi;
                decision.linear.x = 0.0;
                decision.angular.z = 0.15;
                while(yaw > target_yaw)
                {
                    velocity.publish(decision);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            else if(which_bumper == 1)
            {
                target_yaw = cur_yaw+pi/2 > pi ? (cur_yaw-1.5*pi) : (cur_yaw+pi/2);
                if(target_yaw > pi-0.02)
                    target_yaw = -pi;
                decision.linear.x = 0.0;
                decision.angular.z = 0.15;
                while(yaw < target_yaw)
                {
                    velocity.publish(decision);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            else
            {
                target_yaw = cur_yaw+pi/4 > pi ? (cur_yaw-1.75*pi) : (cur_yaw+pi/4);
                if(target_yaw > pi-0.02)
                    target_yaw = pi;
                decision.linear.x = 0.0;
                decision.angular.z = 0.15;
                while(yaw < target_yaw)
                {
                    velocity.publish(decision);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            decision.linear.x = 0.2;
            decision.angular.z = 0.0;
            start = ros::Time::now();
            while(ros::Time::now()-start < ros::Duration(3.0))
            {
                velocity.publish(decision);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if(!move_forward)
                    break;
            }
            bump = false;
        }
    }
}

void AutoNav::pilot(const ros::TimerEvent& time)
{   
    //test the linear accelerate/decelerate functions
    if(acc_or_not && DRIVE)
    {
        printf("accelerate\n");
        linear_accelerate(time, -0.16, 5.0);
        printf("keep the speed\n");
        ros::Time start = ros::Time::now();
        geometry_msgs::Twist decision;
        decision.linear.x = -0.16;
        decision.angular.z = 0.0;
        while(ros::Time::now() - start <= ros::Duration(4.0))
        {
            velocity.publish(decision);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
        printf("decelerate\n");
        linear_decelerate(time, -0.16, 5.0);
        printf("stop\n");
        acc_or_not = false;
    }

    //test the angular accelerate/decelerate functions
    if(acc_or_not && DRIVE)
    {
        printf("accelerate\n");
        angular_accelerate(time, 0.5, 5.0);
        printf("keep the speed\n");
        geometry_msgs::Twist decision;
        decision.linear.x = 0.0;
        decision.angular.z = 0.5;
        ros::Time current_time = ros::Time::now();
        while(ros::Time::now() - current_time < ros::Duration(4.0))
        {
            velocity.publish(decision);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
        printf("decelerate\n");
        angular_decelerate(time, 0.5, 5.0);
        printf("stop\n");
        acc_or_not = false;
    }



    /*******************Don't delete this code *********************/
    /*if(leave_docking_station)
    {
        leave_station_action(time);
    }
    else if(half_battery && near_docking_station)
    {
        auto_docking_action(time);
    }
    else if(battery_is_low == false)
    {
        battery_is_good_action(time);
    }
    else if(near_docking_station)
    {
        std::cout<<"auto docking"<<std::endl;
        auto_docking_action(time);
    }
    else
    {
        //battery is low (navigate to the docking station)
        std::cout<<"move to docking station"<<std::endl;
        ros::Time start = ros::Time::now();
        while(ros::Time::now()-start < ros::Duration(5.0)){//do nothing, just to waste the time}
        battery_is_low_action(time);
    }*/
}
