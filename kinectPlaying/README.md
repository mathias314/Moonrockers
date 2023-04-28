This folder contains scripts and an additional ROS workspace for working with the Kinect obstacle detection algorithm.

The catkin_ws contains the Azure_Kinect_ROS_Driver as a git submodule. You can include this submodule to include that source.
To build and run use the standard catkin_make and source devel/setup.sh. Then run roslaunch azure_kinect_ros_driver driver.launch to start up the ROS driver.
If you have not yet gotten the Kinect working on the Jetson/Ubuntu 18.04, see the following page for instructions on how to do so. This is a prerequisite to getting the ROS driver working. https://gitlab.com/sdmines/projects/moonrockers/2022/production/-/wikis/Building-the-Azure-Kinect-SDK-on-Ubuntu-18.04

Running the obstacleDetect.py script will get output from the /depth/image_raw topic published by the driver from the camera, and generate the output image and list of points that are determined to be obstacles. The output can be investigated with the window that pops up, along with the reconstructed image and 2nd derivative image, as well as the output matrix csv.

For more details on the implementation of this script, see the system document.

Here is a link to the ROS Kinect Driver repo: https://github.com/microsoft/Azure_Kinect_ROS_Driver
