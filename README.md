The main code for this project is in directory: src/turtlebot/src/navigation.cpp

To run this code, at first, you need to make sure that the network configuration is already set up between your PC and the turtlebot.
Here is a good reference about how to get this thing done: http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration



-------------------------------------

HOW TO RUN THIS CODE:

On the turtlebot:

You need to bring up the turtlebot at first, open two terminal, and run

    $roslaunch turtlebot_bringup minimal.launch --screen

    $roslaunch turtlebot_bringup 3dsensor.launch

in each of them.

On PC:

1. Git clone the repository, DON'T change the name of the folder. 

    $git clone https://gitlab.engr.illinois.edu/xzhan140/catkin_ws

2. Rename the folder name from turtlebot to catkin_ws. We are using CMake, all the code must be in the catkin_ws folder.

3. Change the directory to the repository folder

    $cd catkin_ws

4. Compile the code

    $catkin_make

5. Run the code

    $rosrun turtlebot navigation

6. To stop the robot just press ctrl+c to break the process


