The main code for this project is in directory: turtlebot/src

To run this code, at first, you need to make sure that your PC has Ubuntu 14.04 OS. And then, you need to make sure that the network configuration is already set up between your PC and the turtlebot.
Here is a good reference about how to get this thing done: http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration .

What is more, you need to install some packages on your PC. Please check the packages_list.txt for details.

Make sure that compressedDepth image_transport plugin is installed on your PC. You can check it by running

    $ rosrun image_transport list_transports

in your terminal. "image_transport/compressedDepth" should show up in your terminal. 


-------------------------------------

HOW TO RUN THIS CODE:

On the turtlebot:

You need to bring up the turtlebot at first, open two terminal, and run

    $ roslaunch turtlebot_bringup minimal.launch --screen

    $ roslaunch turtlebot_bringup 3dsensor.launch

in each of them.

On PC:

1. Git clone the repository. 

        $ git clone https://github.com/bengrosser/autonomous-video-artist

2. Create a workspace for catkin on your PC. If you are interested in it, here is a catkin tutorial: http://wiki.ros.org/catkin/Tutorials.

        $ source /opt/ros/indigo/setup.bash

        $ mkdir -p ~/catkin_ws/src

        $ cd ~/catkin_ws/

        $ catkin_make

3. Create a turtlebot package in the workspace

        $ cd ~/catkin_ws/src

        $ catkin_create_pkg turtlebot std_msgs rospy roscpp

4. Place the files in turtlebot folder in this repo to directory /catkin_ws/src/turtlebot in your PC.

5. Change the directory to the repository folder

        $ cd catkin_ws

6. Compile the code

        $ catkin_make

7. Source the setup bash

        $ source devel/setup.bash

8. Run the code

    To run the automatic navigation with point cloud (including position tracking and battery information):
    
        $ rosrun turtlebot navigation

    To run the automatic navigation with depth image (Please make sure that compressedDepth image_transport plugin is installed on your system):

        Open a terminal on the PC, run:

            $ roslaunch kobuki_auto_docking minimal.launch --screen

        Then open another terminal tab on the PC, run:

            $ rosrun turtlebot depth_nav _image_transport:=compressedDepth

    To run the auto charging code:
       
        Open a terminal on the PC, run:

            $ roslaunch kobuki_auto_docking minimal.launch --screen

        Then open another terminal tab on the PC, run:

            $ rosrun turtlebot charging

9. To stop the robot just press ctrl+c to break the process
