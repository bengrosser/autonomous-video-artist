The code in this directory should be on your turtlebot.

To run this code, you need to make sure that the network configuration is already set up between your PC and the turtlebot. Here is a good reference about how to get this thing done: http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration .

And also, make sure the TIME SYNCHRONIZATION is CORRECT at your robot. If it is wrong, you can manually synchronize the time by:
	
	$ sudo apt-get install chrony
	
	$ sudo ntpdate ntp.ubuntu.com

-----------------------------------------------

HOW TO BRINGUP YOUR TURTLEBOT:

Open two terminal tabs on your turtlebot, and run:

    $ roslaunch turtlebot_bringup minimal.launch --screen

    $ roslaunch turtlebot_bringup 3dsensor.launch

in each of the tab.

Please check usb_cam directory to setup multiple usb cams on your turtlebot.

Please check sys_info directory to send system information to the PC.
