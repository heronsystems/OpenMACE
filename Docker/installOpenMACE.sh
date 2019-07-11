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

installTools() {
    echo "************************************"
    echo "Installing OpenMACE tools..."
    echo "************************************"

    cd $MACE_ROOT
    git submodule init
    git submodule update

    # Install Flann:
    cd $MACE_ROOT/tools/flann
    BUILD_DIRECTORY="${MACE_ROOT}/tools/flann/build"
    if [ ! -d "$BUILD_DIRECTORY" ]; then
        mkdir build
    fi
    cd ./build
    CXXFLAGS=-std=c++11 cmake ..
    make
    make install

    # Install Octomap:
    cd $MACE_ROOT/tools/octomap
    BUILD_DIRECTORY="${MACE_ROOT}/tools/octomap/build"
    if [ ! -d "$BUILD_DIRECTORY" ]; then
        mkdir build
    fi
    cd ./build
    cmake ..
    make
    make install

    # Install libccd:
    cd $MACE_ROOT/tools/libccd
    BUILD_DIRECTORY="${MACE_ROOT}/tools/libccd/build"
    if [ ! -d "$BUILD_DIRECTORY" ]; then
        mkdir build
    fi
    cd ./build
    cmake -G "Unix Makefiles" ..
    make
    make install

    # Change directory back to MACE_ROOT
    cd $MACE_ROOT
}


installMACE() {
    echo "************************************"
    echo "Installing MACE..."
    echo "************************************"

    echo "Setting OpenMACE environment variables..."
    echo "export MACE_ROOT=${MACE_ROOT}" >> ~/.bashrc
    source ~/.bashrc

    echo "${MACE_ROOT}/lib" >> /etc/ld.so.conf.d/OpenMACE.conf

    ROS_DIRECTORY="/opt/ros/kinetic"
    if [ "$installROS" = "1" ] || [ -d "$ROS_DIRECTORY" ]; then
        cd $MACE_ROOT/catkin_sim_environment
        . /opt/ros/kinetic/setup.bash
        catkin_make
        catkin_make
        catkin_make
        echo "source $(MACE_ROOT)/catkin_sim_environment/devel/setup.bash" >> ~/.bashrc
        source ~/.bashrc
    fi

    cd $MACE_ROOT
    if [ "$cleanFirst" = "1" ]; then
        rm -rf build
        rm -rf bin
        rm -rf lib
        rm -rf include
    fi

    # Make new directories if they don't exist:
    BUILD_DIRECTORY="${MACE_ROOT}/build"
    if [ ! -d "$BUILD_DIRECTORY" ]; then
        mkdir build
    fi
    BIN_DIRECTORY="${MACE_ROOT}/bin"
    if [ ! -d "$BIN_DIRECTORY" ]; then
        mkdir bin
    fi
    LIB_DIRECTORY="${MACE_ROOT}/lib"
    if [ ! -d "$LIB_DIRECTORY" ]; then
        mkdir lib
    fi
    INCLUDE_DIRECTORY="${MACE_ROOT}/include"
    if [ ! -d "$INCLUDE_DIRECTORY" ]; then
        mkdir include
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
echo "MACE_ROOT is set to: ${MACE_ROOT}"


USERNAME=$(logname)
echo "Non-root username to install MACE: ${USERNAME}"

installPrereqs=
installROS=
installTools=
cleanFirst=

while [ "$1" != "" ]; do
    case $1 in
        -p | --prereqs ) installPrereqs=1
                         ;;
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

# If toggled, install prerequisites 
if [ "$installPrereqs" = "1" ]; then
    installPrereqs
fi

# If toggled, install ROS kinetic
if [ "$installROS" = "1" ]; then
    installROS
fi

# If toggled, install Tools as non-root user
if [ "$installTools" = "1" ]; then
    # sudo -u $USERNAME installTools
    installTools
fi


# Install MACE as non-root user
# sudo -u $USERNAME installMACE
installMACE

cd $MACE_ROOT