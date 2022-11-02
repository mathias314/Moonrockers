# ROS `src` Directory

This directory contains the source for ROS nodes that run on the Jetson and the control room laptop. The device the module is intended to run on is indicated after the name. See [/ros/launch/README.md](/ros/launch/README.md) for details on how to run the robot using these nodes.

## `design_fair_demo`: mixed
Contains an ArUco tag tracking demo that was first used at the 2022 Senior Design Fair to show off some basic computer vision capabilities. `demo.py` should run on the robot and `gui.py` should run on the laptop.

## `drive_manager`: robot
Takes inputs from different sources and decides where to drive the robot. Talks to the ROS nodes on the Arduino.

## `gamepad`: laptop
Reads the inputs from a USB gamepad and publishes the ones we care about.

## `gamepad_msgs`: mixed
Contains ROS `.msg` files related to gamepad events.

## `gui`: laptop
Displays the current status of the robot and gamepad. Allows click-to-drive navigation. *Still very much under construction.*

## `moon_msgs`: mixed
Various custom ROS `.msg` files used in this project.

## `poser`: robot
Uses a USB camera and ArUco tag(s) to determine the location of the robot relative to said tag(s).