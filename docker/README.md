# ROS Docker Container
This directory contains (almost) everything you need to run the custom ROS Docker container on your laptop. This makes it easy to run the control room side ROS code without worrying about version compatibility between your OS and different software versions. The one big prerequisite to meet: you MUST be using a relatively recent Linux-based operating system. I (@7314739) was running the container under Ubuntu 21.10 as of spring 2022, but it should work on versions that are a few years old as well.

## Installing Docker
Follow the instructions on [this page](https://docs.docker.com/engine/install/) to install the Docker Engine. Click on the name of your distribution in the table under the Server heading. If you're using some variation of Ubuntu (Lubuntu, Xubuntu, etc), select Ubuntu. I recommend following the "Install using the repository" instructions. You do **not** need to follow step 2 of "Install Docker Engine", step 1 will grab the default version for you.

After installing, follow the instructions for "Manage Docker as a non-root user" on [this page](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

## Notation Used
If you see something like this:

`$ echo "hello world"`

It means you're running a command directly on your local machine.

If you see something like this:

`docker$ echo "hello world"`

It means you're running a command inside the Docker container.

## First-Run Setup
Build the image the container will use:

`$ ./build.sh`

## Running The Container
Start the container:

`$ ./run.sh`

## Start ROS
You probably want to start some ROS interface to the robot on the laptop (within the container) at this point. See [the main ROS README](/ros/README.md) and the [`launch` directory README](/ros/launch/README.md) for more details.

Navigate to the `ros` directory:

`docker$ cd workspace/production/ros`

Set up the ROS environment:

`docker$ source devel/setup.sh`

Run the `.launch` file you want:

`docker$ roslaunch launch/base/drive.launch`

## Exiting The Container
When you are at an empty command line (no program running in the terminal, nothing typed in), press `CTRL-D`.