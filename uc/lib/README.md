# lib  
This directory contains project specific (private) libraries.
PlatformIO compiles them to static libraries and link into executable file.

 - **main** - The main library for the project. Dumping ground for everything it doesn't make sense to split out.  
 - **Serial_CAN_Module** - Library for the CAN module used for communicating with the MCs. This is a modified version of the lib found here: https://github.com/Longan-Labs/Serial_CAN_Arduino   
 - **FRCMotorControllers** - Library for interfacing with CTR and REV motor controllers.  
 - **TestChassis** - Lib for interfacing with dumb motors on test chassis.  
 - **ros_lib** - Files for the ROS communications with the Jetson. These files are generated from ROS.  
