This is the code for turtlebot to publish the system information.

HOW TO RUN THIS CODE:

On the turtlebot:

1. Git clone the repository.
        
        $ git clone https://github.com/bengrosser/autonomous-video-artist

2. Create a workspace on your turtlebot.
        
        $ source /opt/ros/indigo/setup.bash

        $ mkdir -p ~/catkin_ws/src

        $ cd ~/catkin_ws/

        $ catkin_make

3. Create a publisher package in the workspace.

        $ cd ~/catkin_ws/src

        $ catkin_create_pkg publisher std_msgs rospy roscpp

4. Place the files in this directory to ~/catkin_ws/src/publisher on your turtlebot.

5. Compile and run the code

        $ cd ~/catkin_ws

        $ catkin_make

        $ source devel/setup.bash

        $ rosrun publisher sys_pub

6. Then on your PC, run 

        $ rostopic list
   
   You can see a new node called /sys/RAM. Run

        $ rostopic echo /sys/RAM

   You can see the content of this node.
