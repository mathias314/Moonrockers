# ROS Root

This is the root directory for all ROS code that runs on the Jetson or the control room laptop. If you're looking for Arduino code, see `../uc`.

## Building the code
Instructions assume the current directory is `ros`.

Run `catkin_make` to build:

`$ catkin_make`

## Directories always here

`src`: contains source files for ROS nodes. See [ros/src/README.md](ros/src/README.md) for more details.

`launch`: contains `.launch` files that start a collection of ROS nodes all at the same time. See [ros/launch/README.md](ros/launch/README.md) for more details.

## Directories created by Catkin
You won't see these on GitLab as they are created by the Catkin build system. If you're having build issues, deleting these two directories may help.

`devel`: main file of interest is `setup.sh`, which will set up your terminal with the ROS environment to allow building and running nodes. Activate it with `$ source devel/setup.sh`. Always run `source devel/setup.sh` to avoid issues with having an improper namespace.

`build`: nothing interesting in here (ideally) from the standpoint of a developer on this project. Contains files used in the process of building ROS nodes' code.