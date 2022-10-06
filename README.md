# ads_rw_usbcam
ROS implementation of a usb camera passing the colour average to a function that uses the modified Beckhoff/ADS library (ADSROS) which passes to a variable in TwinCAT. Requires a USB camera, I used a Logitech C920 camera.

Currently (2022-10-04) tested with:
===================================

host (amd64)     | target| compiler
-----------------|-------|-------------
 ubuntu 20.04    | win64 | gcc 9.4.0

Prepare your client to run this library
======================================

- follow the steps to run the ADSROS ROS node (https://github.com/Dshieff/ADSROS)

- in your NotificationTest TwinCAT PLC project add a new variable in Main called writeVar

- install these ROS packages:
  - sudo apt-get install ros-(ROS version name)-cv-bridge
  - sudo apt-get install ros-(ROS version name)-image-transport

- navigate into the catkin_ws/src directory

- git clone the ads_rw_usbcam repo into the src folder of catkin workspace:
git clone https://github.com/Dshieff/ads_rw_usbcam.git <directory>

- build the ads_rw_usbcam package (the version of python is 3.10, can be different)
catkin build
or if that does not work:
catkin build <package name> -DPYTHON_EXECUTABLE=/usr/bin/python3 
-DPYTHON_INCLUDE_DIR=/usr/local/include/python3.10 

- If there is a weird error and catkin does not build try this:
catkin clean

- Once the package is built, source the files so that the packages in the workspace recognisable to the rest of the system
source ~/catkin_ws/devel/setup.bash

- get ros to run:
source /opt/ros/noetic/setup.bash
roscore

- find the ROS package, make sure that ROS replies witht the file path of the ads library:
rospack find ads_rw_usbcam

- make sure that your TwinCAT NotificationTest project is running on your Target!

- launch the launch file: roslaunch ads_rw_usbcam ads_rw_usbcam.launch
