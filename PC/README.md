The code in this directory should be on your PC.

To run this code, at first, you need to make sure that your PC has Ubuntu 14.04 OS. And then, you need to make sure that the network configuration is already set up between your PC and the turtlebot. Here is a good reference about how to get this thing done: http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration .

What is more, you need to install some packages on your PC. Please check the packages_list.txt for details.

Make sure that compressedDepth image_transport plugin is installed on your PC. You can check it by running

    $ rosrun image_transport list_transports

in your terminal. "image_transport/compressedDepth" should show up in your terminal. 

After these, your should bringup your turtlebot at first. Please check the robot directory https://github.com/bengrosser/autonomous-video-artist/tree/master/robot.


-------------------------------------

Make sure that you can ssh to the turtlebot without password at first.

Change the IP address in autonomous_video_artist/src/videos.cpp to the IP address of the turtlebot.

-------------------------------------

HOW TO RUN THIS CODE:

At first, please check autonomous-video-artist/robot directory to bring up your turtlebot.

On PC:

1. Git clone the repository. 

        $ git clone https://github.com/bengrosser/autonomous-video-artist

2. Create a workspace for catkin on your PC. If you are interested in it, here is a catkin tutorial: http://wiki.ros.org/catkin/Tutorials.

        $ source /opt/ros/indigo/setup.bash

        $ mkdir -p ~/catkin_ws/src

        $ cd ~/catkin_ws/

        $ catkin_make

3. Create a turtlebot package in the workspace.

        $ cd ~/catkin_ws/src

        $ catkin_create_pkg autonomous_video_artist std_msgs rospy roscpp

4. Place the files in PC/autonomous_video_artist folder in this repo to directory ~/catkin_ws/src/autonomous_video_artist in your PC.

5. Change the directory to the repository folder.

        $ cd catkin_ws

6. Compile the code.

        $ catkin_make

7. Source the setup bash.

        $ source devel/setup.bash

8. Run the code

    Make sure that you have bringup the robot. Please check https://github.com/bengrosser/autonomous-video-artist/tree/master/robot.

    Then run

        $ rosrun autonomous_video_artist ava
    
    on the PC.

9. To stop the robot just press ctrl+c to break the process.
