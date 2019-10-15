#!/bin/bash

usage() {
    echo -n "./installPrereqs.sh [OPTION]
    This is an install script for OpenMACE prerequisites. Pass in flags to choose which pieces to install.

    ${bold}Options:${reset}
    -r, --ros           Toggle install ROS flag
    -h, --help          Display this help and exit
"
}

installPrereqs() {
    echo "************************************"
    echo "Installing OpenMACE prerequisites..."
    echo "************************************"

    apt-get update

    # Install tools here, (recommended to use multiple lines so they don't have to all reinstall if you change one)
    apt-get install -y cmake
    apt-get install -y nano
    apt-get install -y tmux
    apt-get install -y git
    apt-get update
    apt-get install -y qt5-default
    apt-get install -y libqt5serialport5-dev
    apt-get install -y build-essential
    apt-get install -y libboost-system-dev
    apt-get install -y python-pip
    apt-get install -y python-dev
    pip install --upgrade pip
    pip install --upgrade virtualenv

    # If we are installing ROS, these will be installed as dependencies of ROS packages
    if [ "$installROS" != "1" ]; then
        apt-get update
        apt-get install -y pkg-config
        apt-get install -y liblz4-dev
    fi
}

installROS() {
    echo "************************************"
    echo "Installing ROS..."
    echo "************************************"

    # May not need this first apt-get update...
    apt-get update
    apt-get install -y lsb-release
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    apt-get update
    apt-get install -y ros-kinetic-desktop-full
    rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    apt-get install -y python-rosinstall
    apt-get install -y python-rosinstall-generator
    apt-get install -y python-wstool
    apt-get install -y ros-kinetic-octomap*
    apt-get install -y ros-kinetic-tf*
    apt-get install -y python-rospkg
    pip install rospkg
}


#### MAIN
installROS=

while [ "$1" != "" ]; do
    case $1 in
        -r | --ros )     installROS=1
                         ;;
        -h | --help )    usage
                         exit
                         ;;
        * )              usage
                         exit 1
    esac
    shift
done

# If toggled, install prerequisites
installPrereqs

# If toggled, install ROS kinetic
if [ "$installROS" = "1" ]; then
    installROS
fi
