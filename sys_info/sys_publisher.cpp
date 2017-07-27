#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sys/sysinfo.h>

int main(int argc, char ** argv)
{
    ros::init(argv, argv, "publisher");
    ros::NodeHandle n;

    ros::Publisher sys_pub = n.advertise<std_msgs::Int32>("sys/RAM", 1000);
    ros::Rate loop_rate(10);

    int count = 0;    
    while(ros::ok())
    {
        struct sysinfo si;
        sysinfo(&si);
        std_msgs::Int32 msg;
        msg.data = si.freeram;
        sys_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        std::cout<<count<<" loop"<<endl;
    }
    
    return 0;
}
