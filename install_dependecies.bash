#!/bin/bash
# This script will install the needed packages for running p3dx simulator

# check ROS version
rosversion=`rosversion -d`
if [[ $rosversion == *"unknown"* ]]
then
    echo "Unknown ROS version!"
    exit
else
    echo "Found ROS version $rosversion"
fi

sudo apt-get install ros-$rosversion-controller-manager
sudo apt-get install ros-$rosversion-robot-localization
sudo apt-get install ros-$rosversion-interactive-marker-twist-server
sudo apt-get install ros-$rosversion-robot-state-publisher
sudo apt-get install ros-$rosversion-twist-mux