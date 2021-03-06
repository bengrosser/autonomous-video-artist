/**********************
* This is the code to start the autonomous-video-artist process.
* 
* Created on 8/9/2017
* Author: Xinyu Zhang
* Contact: xzhan140@illinois.edu
*
***********************/


#include "AutoNav.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle node("autoNav");
    
    node.setParam("drive", true);
    node.setParam("drive_linearspeed", 0.16);
    node.setParam("drive_angularspeed", 0.5);
	node.setParam("capture_duration", 30);
	

    AutoNav turtlebot(node);


    node.deleteParam("drive");
    node.deleteParam("drive_linearspeed");
    node.deleteParam("drive_angularspeed");
	node.deleteParam("capture_duration");

    return 0;
}
