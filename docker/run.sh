#!/bin/bash

# find the file used for X authentication
# this works on my (Dustin's) laptop with Ubuntu 21.10 and has not been extensively tested
# if you can't get display output from the Docker container, it likely has something to do
#  with this.
# show xauth info | get the correct line | extract a file path from the end of the line
xauthfile=$(xauth info | grep "Authority file" | grep -o "/.*")

# get the current IP address in the 192.168.0.* range, this is what the router's
#  DHCP server should be configured for
# list out IPs | find 192.168.0.* with a trailing slash | remove the trailing slash
ip=$(ip a | grep -E -o 192\.168\.0\.[0-9]{3}/ | grep -o [^/]*)

# the IP address of the Jetson along with the port that roscore is running on
jetson_ip="192.168.0.100:11311"

# make the workspace directory if it doesn't exist, this is the Docker container's
#  persistent read-write storage and will be mounted with the -v option below
if [ ! -e workspace ]
then
  mkdir workspace
fi

# grab the output of pwd so we don't have to run it a bunch of times
wd=$(pwd)

# run the Docker image
docker run -it \
\
`### mount some useful directories` \
-v ${wd}/workspace:/workspace \
-v ${wd}/..:/workspace/production \
\
`### for USB forwarding the gamepad` \
--privileged -v /dev/bus/usb:/dev/bus/usb \
\
`### ROS IP environment variables so this laptop can talk to the Jetson` \
`### -e "ROS_MASTER_URI=http://${jetson_ip}"` \
`### -e "ROS_IP=${ip}"` \
\
`### this is for X forwarding so we can run GUIs in the container` \
-e DISPLAY \
--net=host \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ${xauthfile}:/root/.Xauthority \
\
`### expose all network ports to the container` \
--expose 0-65535 \
\
`### run bash in the container` \
ros-noetic-moonrockers bash
