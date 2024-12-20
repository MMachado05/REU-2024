#! /bin/bash

# Detect ROS version
if [ -e /opt/ros/indigo/setup.bash ]; then
  echo "Detected ROS Indigo."
  source /opt/ros/indigo/setup.bash
elif [ -e /opt/ros/kinetic/setup.bash ]; then
  echo "Detected ROS Kinetic."
  source /opt/ros/kinetic/setup.bash
elif [ -e /opt/ros/melodic/setup.bash ]; then
  echo "Detected ROS Melodic."
  source /opt/ros/melodic/setup.bash
elif [ -e /opt/ros/noetic/setup.bash ]; then
  echo "Detected ROS Noetic."
  source /opt/ros/noetic/setup.bash
else
  echo "Failed to detected ROS version."
  exit 1
fi

# Setup apt-get
echo "Adding Dataspeed server to apt..."
sudo sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 66F84AE1EB71A8AC108087DCAF677210FF6D3CDA
sudo apt-get update

# Setup rosdep
echo "Setting up rosdep..."
if [ -z "$ROS_DISTRO" ]; then
  echo "Error! ROS not detected. Not updating rosdep!"
else
  sudo sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
  rosdep update --rosdistro=$ROS_DISTRO
fi

sudo apt-get install -y ros-$ROS_DISTRO-dbw-polaris
sudo apt-get upgrade

echo "SDK install: Done"

