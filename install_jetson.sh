#!/bin/bash

## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt update
## Get ROS/Gazebo
sudo apt install ros-melodic-ros-base ros-melodic-usb-cam ros-melodic-joy -y
sudo apt install python-opencv python-pandas python-numpy python-cv-bridge -y
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource
## Get rosinstall
sudo apt install python-rosinstall -y

## Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Install dependencies
sudo apt install python-rosinstall-generator python-catkin-tools -y

## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source

if [[ ! -z $unsupported_os ]]; then
    >&2 echo -e "\033[31mYour OS ($unsupported_os) is unsupported. Assumed an Ubuntu 18.04 installation,"
    >&2 echo -e "and continued with the installation, but if things are not working as"
    >&2 echo -e "expected you have been warned."
fi

cd ~/catkin_ws/src
git clone https://github.com/halitgurpinar/angelshark --recursive

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=melodic --os=ubuntu:bionic -y

catkin_make