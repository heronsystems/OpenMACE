#!/bin/bash

usage() {
    echo -n "./install_test_help.sh [OPTION]
    This is an install script for OpenMACE. Pass in flags to choose which pieces to install.

    ${bold}Options:${reset}
    -p, --prereqs       Toggle install OpenMACE prerequisits flag
    -r, --ros           Toggle install ROS flag
    -t, --tools         Toggle install OpenMACE tools flag (typically done for initial installs from scratch)
    -c, --clean         Toggle clean first before building OpenMACE
    -h, --help          Display this help and exit
"
}

installPrereqs() {
    echo "\nInstalling OpenMACE prerequisites...\n"

    apt-get update

    # Install tools here, (recommended to use multiple lines so they don't have to all reinstall if you change one)
    apt-get install -y qt5-default cmake
    apt-get install -y git
    apt-get install -y libqt5serialport5-dev
    apt-get install -y build-essential
    apt-get install -y nano
    apt-get install -y tmux
    apt-get install -y python-pip python-dev build-essential
    pip install --upgrade pip
    pip install --upgrade virtualenv
}

installROS() {
    echo "\nInstalling ROS...\n"

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
    apt-get install -y libboost-system-dev
    apt-get install -y ros-kinetic-octomap*
    apt-get install -y ros-kinetic-tf*
    apt-get install -y python-rospkg
    pip install rospkg
}

installTools() {
    echo "\nInstalling OpenMACE tools...\n"

    cd $MACE_ROOT
    git submodule init
    git submodule update

    # Install Flann:
    cd $MACE_ROOT/tools/flann
    mkdir build
    cd ./build
    cmake ..
    make
    make install

    # Install Octomap:
    cd $MACE_ROOT/tools/octomap
    mkdir build
    cd ./build
    cmake ..
    make
    make install

    # Install libccd:
    cd $MACE_ROOT/tools/libccd
    mkdir build
    cd ./build
    cmake -G "Unix Makefiles" ..
    make
    make install

    # Change directory back to MACE_ROOT
    cd $MACE_ROOT
}


installMACE(cleanFirst, installROS) {
    echo "\nInstalling MACE...\n"

    echo "$(MACE_ROOT)/lib" > /etc/ld.so.conf.d/OpenMACE.conf

    if [ "$installROS" = "1" ]; then
        cd $MACE_ROOT/catkin_sim_environment
        . /opt/ros/kinetic/setup.bash
        catkin_make
        echo "source $(MACE_ROOT)/catkin_sim_environment/devel/setup.bash" >> ~/.bashrc
        source ~/.bashrc
    fi

    cd $MACE_ROOT
    if [ "$cleanFirst" = "1" ]; then
        rm -rf build
        mkdir build
    fi

    cd $MACE_ROOT/build
    qmake ../src/src.pro
    make
    make install
    ldconfig
}


#### MAIN
# Set MACE_ROOT environment variable:
maceRootPath="$(pwd)"
export MACE_ROOT=$maceRootPath
echo $MACE_ROOT

installPrereqs=
installROS=
installTools=
cleanFirst=

while [ "$1" != "" ]; do
    case $1 in
        -p | --prereqs ) installPrereqs=1
        -r | --ros )     installROS=1
                         ;;
        -t | --tools )   installTools=1
                         ;;
        -c | --clean )   cleanFirst=1
                         ;;
        -h | --help )    usage
                         exit
                         ;;
        * )              usage
                         exit 1
    esac
    shift
done

if [ "$installPrereqs" = "1" ]; then
    #installPrereqs
fi

if [ "$installROS" = "1" ]; then
    #installROS
fi

if [ "$installTools" = "1" ]; then
    #installTools
fi


#installMACE($cleanFirst, $installROS)

cd $MACE_ROOT