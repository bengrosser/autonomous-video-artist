#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h>

using namespace std;

class AutoDocking
{
private:
    ros::NodeHandle node;
    
    

public:
    AutoDocking(ros::NodeHandle& handle):node(handle){
        actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> client("dock_drive_action",true);
        client.waitForServer();
        kobuki_msgs::AutoDockingGoal goal;
        actionlib::SimpleClientGoalState dock_state = actionlib::SimpleClientGoalState::LOST;
        client.sendGoal(goal);
        ros::Time time = ros::Time::now();
        while(!client.waitForResult(ros::Duration(3))){
            dock_state = client.getState();
            ROS_INFO("Docking status: %s", dock_state.toString().c_str());
            
            if(ros::Time::now() > (time+ros::Duration(500))){
                ROS_INFO("Docking took more than 500 seconds, canceling.");
                client.cancelGoal();
                break;
            }
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "charging");
    ros::NodeHandle node("autoDocking");
    AutoDocking turtlebot(node);
    return 0;
}
