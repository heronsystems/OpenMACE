#!/bin/bash

usage() {
    echo -n "./installOpenMACE.sh [OPTION]
    This is an install script for OpenMACE. Pass in flags to choose which pieces to install.

    ${bold}Options:${reset}
	-s, --catkin		Toggle the compilation of the catkin environment within the OpenMACE repository
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

requestROSExtension()
{
    echo "*******************************************"
    echo "Setting up the ROS_ROOT_DIR extension"
    echo "*******************************************"

    #There is no ROS_ROOT_DIR defined
    echo "Enter your ROS version:"
    read rosVersion
    echo "export ROS_ROOT_DIR=/opt/ros/${rosVersion}" >> ~/.bashrc
    _ROS_ROOT_DIR="/opt/ros/${rosVersion}"
    echo "ROS_ROOT_DIR=\"${_ROS_ROOT_DIR}\"" >> /etc/environment
    source ~/.bashrc
}

compileCatkin(){
    echo "*******************************************"
    echo "Compiling the catkin environment directory"
    echo "*******************************************"

	ROS_DIRECTORY="${ROS_ROOT_DIR}"
    if [ ! -d "$ROS_DIRECTORY" ]; then
		requestROSExtension
    fi
	
    if [ -d "$ROS_DIRECTORY" ]; then
		rosBasePath=`dirname "$ROS_DIRECTORY"`
		rosVersion=`basename "$ROS_DIRECTORY"`

        cd $MACE_ROOT/catkin_sim_environment
        . /opt/ros/$rosVersion/setup.bash
        catkin_make
        catkin_make
        catkin_make
		if [ "$sourceCatkin" = "1" ]; then
        	  echo "source ${MACE_ROOT}/catkin_sim_environment/devel/setup.bash" >> ~/.bashrc
        	  source ~/.bashrc
		else
		  echo "The user has selected not to update the catkin_sim_environment path in the bashrc file!!!"
		fi
    fi

    echo "*****************************************"
	echo "FINISHED COMPILING THE CATKIN DIRECTORY"
    echo "*****************************************"
}

installMACE() {
    echo "************************************"
    echo "Installing MACE..."
    echo "************************************"

    echo "Setting OpenMACE environment variables..."

	if [ "$setMACEPath" = "1" ]; then
	  echo "export MACE_ROOT=${MACE_ROOT}" >> ~/.bashrc
    	  source ~/.bashrc
	fi

    echo "${MACE_ROOT}/lib" >> /etc/ld.so.conf.d/OpenMACE.conf


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
    qmake DEFINES+="WITH_HERON_MAVLINK_SUPPORT" ../src/src.pro
    make
    make install
    # ldconfig
}

installGUI() {
    echo "************************************"
    echo "Installing MACE GUI..."
    echo "************************************"

    cd $MACE_ROOT/MACE_Frontend
    yarn
}


#### MAIN
# Set MACE_ROOT environment variable:
setMACEPath=0
if [[ -z "${MACE_ROOT}" ]]; then
  setMACEPath=1
  currentMACEPath="$(pwd)"
  export MACE_ROOT=$currentMACEPath
  echo "MACE_ROOT is now set to: ${MACE_ROOT}"
fi

if [ "$setMACEPath" = "0" ]; then
  echo "MACE_ROOT is already set to: $MACE_ROOT"
  currentMACEPath="$(pwd)"
  bashMACEPath="${MACE_ROOT}"
  echo "The current path is $currentMACEPath"
  echo "The bash MACE path is $bashMACEPath"
fi

setROSPath=0
if [[ -z "${ROS_ROOT_DIR}" ]]; then
  setROSPath=1
  echo "ROS_ROOT_DIR has not been found, will be setting this up!!!"
fi

if [ "$setROSPath" == "0" ]; then
  echo "ROS_ROOT_DIR is already set to: ${ROS_ROOT_DIR}"
fi

USERNAME=$(logname)
echo "Non-root username to install MACE: ${USERNAME}"

installGUI=
cmpCatkin=
installTools=
cleanFirst=

while [ "$1" != "" ]; do
    case $1 in
        -s | --catkin )  cmpCatkin=1
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


# If toggled, there is no ROS Path
if [ "$setROSPath" = "1" ]; then
    requestROSExtension
fi

# If toggled, install Tools as non-root user
if [ "$installTools" = "1" ]; then
    installTools
fi

# If toggled, compile the catkin environment
if [ "$cmpCatkin" = "1" ]; then
    compileCatkin
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
