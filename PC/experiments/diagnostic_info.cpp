#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <diagnostic_msgs/DiagnosticStatus.h>

using namespace std;

void diagnostic(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg){
    //std::cout<<"testtest"<<std::endl;
    diagnostic_msgs::DiagnosticStatus ss = msg->status[13];
    std::cout<<ss.name<<std::endl;
    std::cout<<ss.message<<std::endl;
    std::cout<<int(ss.level)<<std::endl;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "diagno");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("/diagnostics_agg", 10, diagnostic);
    ros::spin();
    return 0;
}
