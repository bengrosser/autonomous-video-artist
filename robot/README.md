The code in this directory should be on your turtlebot.

You need to make sure that gstreamer is installed on the robot.
How to install gstreamer:
	
	$ sudo add-apt-repository universe
	$ sudo add-apt-repository multiverse
	$ sudo apt-get update
	$ sudo apt-get install gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
	$ sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev
	$ sudo apt-get install gstreamer-tools

----------------------------------------------

To run this code, you need to make sure that the network configuration is already set up between your PC and the turtlebot. Here is a good reference about how to get this thing done: http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration .

And also, make sure the TIME SYNCHRONIZATION is CORRECT at your robot. If it is wrong, you can manually synchronize the time by:
	
	$ sudo apt-get install chrony
	
	$ sudo ntpdate ntp.ubuntu.com

-----------------------------------------------

If you are using astra camera, make sure that you have installed the driver for astra camera, check this link: https://github.com/tfoote/ros_astra_camera.

-----------------------------------------------

HOW TO BRINGUP YOUR TURTLEBOT:

Open two terminal windows,

on the first window, run

    $ roslaunch turtlebot_bringup minimal.launch --screen

on the second window, if you are using Asus Xtion Pro sensor, run

    $ roslaunch turtlebot_bringup 3dsensor.launch
    
if you are using astra sensor, run

    $ roslaunch astra_launch astra.launch

Please check usb_cam directory to setup multiple usb cams on your turtlebot.

Please check sys_info directory to send system information to the PC.
