# ROS Launch Files

This directory is home to `.launch` files, which are used with `roslaunch` to start a specific combination of ROS nodes. This project's collection of `.launch` files is split into two directories: `base` and `robot`.

`.launch` files found in `base` are intended to be run on the control room computer, probably a laptop with a gamepad connected which will run the monitoring GUI.

`.launch` files found in `robot` are intended to be run on the robot's Jetson and will do things like communicate with the Arduino, run OpenCV, and process LIDAR data.

This is not to say you couldn't run a `.launch` from `base` on the robot. The most practical application for this (that @7314739 can think of) is by running [base/drive-gamepad.launch](ros/base/drive-gamepad.launch) on the robot to allow driving the robot without a laptop by connecting the gamepad straight to the Jetson. *Actually starting the launch file without a laptop is an entirely separate issue.*

If you add a `.launch` here and don't document it in this README, @7314739 will hunt you down and hurt you.

## Running a `.launch` file

Instructions assume that `ros` is the current directory.

If you haven't already run `catkin_make` to build the code and supporting bits and pieces, read the section **Building the code** in [the main ROS README](/ros/README.md). In addition to building the code, this will create the `devel` directory to allow you to do the next step.

Source the setup script from `devel` to set up the ROS environment:

`$ source devel/setup.sh`

Launch your chosen `.launch` file, for example `drive-gamepad.launch`:

`$ roslaunch launch/base/drive-gamepad.launch`

This will start the gamepad input processing code and should be run on the laptop with a gamepad connected.

## `.launch` files in the `base` directory
These `.launch` files will generally be run on the laptop.

| Name                       | Notes                                  | Pairs Nicely With |
|----------------------------|----------------------------------------| ------------------|
| `drive-gamepad.launch`     | Only processes gamepad input and sends to robot. Must have gamepad plugged in. As of commit 19278e8, doesn't like to stop with CTRL-C. Unplug gamepad to stop. See issue #69.| `robot/tag-tracking-demo.launch` |
| `tag-tracking-demo-base.launch` | Starts up a GUI that shows the robot's current position relative to the detected ArUco tag and has a slider to set the follow distance. Check the checkbot to enable tag following. Otherwise, sends gamepad input to the robot. Has the same flaw with not wanting to stop with CTRL-C as `drive-gamepad.launch`. See issue #69. | `robot/tag-tracking-demo.launch` |


## `.launch` files in the `robot` directory
These `.launch` files will generally be run on the robot.

| Name                       | Notes                                  | Pairs Nicely With |
|----------------------------|----------------------------------------| ------------------|
| `tag-tracking-demo-robot.launch` | Starts up nodes to determine the pose of the robot relative to an ArUco tag, take gamepad input, and control the wheels, collection system, and the delivery bucket. Will attempt to point towards and maintain the prescribed distance from the tag if the checkbox in the GUI from `base/tag-tracking-demo-base.launch` is checked. | `base/drive-gamepad.launch` `base/tag-tracking-demo-base.launch`|