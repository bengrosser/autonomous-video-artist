This is a launch file to bringup multiple webcam.

To use this launch file, please make sure that usb_cam package is installed on your turtlebot. If the package is not installed, please run these commands in your terminal:

    $ sudo apt-get update
    $ sudo apt-get install ros-indigo-usb-cam

After installing this package, change directory to usb_cam by

    $ roscd usb_cam

Put multi_usb_cam.launch in the launch folder under directory of usb_cam.

Then, you can start your usb cam on the turtlebot by running 

    $ roslaunch usb_cam multi_usb_cam.launch.

If you meet usb_cam VIDIOC_streamon error 28, please check this link: http://answers.ros.org/question/12582/usb_cam-vidioc_streamon-error-28/ .
