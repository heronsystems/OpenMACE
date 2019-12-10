#!/bin/bash

usage() {
    echo -n "./installOpenMACE.sh [OPTION]
    This is an install script for OpenMACE. Pass in flags to choose which pieces to install.

    ${bold}Options:${reset}
    -r, --ros           Toggle install ROS flag
    -t, --tools         Toggle install OpenMACE tools flag (typically done for initial installs from scratch)
    -c, --clean         Toggle clean first before building OpenMACE
    -h, --help          Display this help and exit
"
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
        echo "source ${MACE_ROOT}/catkin_sim_environment/devel/setup.bash" >> ~/.bashrc
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
    # ldconfig
}

installGUI() {
    echo "************************************"
    echo "Installing MACE GUI..."
    echo "************************************"

    cd $MACE_ROOT/ElectronGUI
    yarn install
    yarn run build:prod
}


#### MAIN
# Set MACE_ROOT environment variable:
maceRootPath="$(pwd)"
export MACE_ROOT=$maceRootPath
echo "MACE_ROOT is set to: ${MACE_ROOT}"


USERNAME=$(logname)
echo "Non-root username to install MACE: ${USERNAME}"

installGUI=
installROS=
installTools=
cleanFirst=

while [ "$1" != "" ]; do
    case $1 in
        -r | --ros )     installROS=1
                         ;;
        -g | --gui )     installGUI=1
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

# If toggled, install Tools as non-root user
if [ "$installTools" = "1" ]; then
    installTools
fi

# If toggled, install GUI
if [ "$installGUI" = "1" ]; then
    installGUI
fi

# Install MACE
installMACE

cd $MACE_ROOT

sudo ldconfig
#printf "\n\nMACE installation finished. \n\n**Make sure to run 'sudo ldconfig' to install newly built libraries**\n\n"
