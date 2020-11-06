# README
#####
# Build & Rebuild with the following command
# sudo docker build -t ubuntu1604:mace .
#####
###
# Now is a good time to learn how to use screen, otherwise the following commands will help you attach/detach and open new terminals
###
### Run and attach with a terminal with:
# sudo docker run -it ubuntu1604:mace
###
# To detach use: Ctrl+p + Ctrl+q
###
# To reattach: sudo docker attach [CONTAINER-ID] (you may have to press enter again to get the command line back)
###
# To start a new bash shell (so you don't interrupt something else you were running)
#     sudo docker exec -ti [CONTAINER-ID] bash
# With a new bash shell run "exit" instead of Ctrl+p + Ctrl+q
#####

## DON'T CHANGE THIS STUFF
#Download base image ubuntu 16.04
FROM ubuntu:18.04

# Update Ubuntu Software repository
RUN apt update
## START CHANGING STUFF

# # Install tools here, (recommended to use multiple lines so they don't have to all reinstall if you change one)
# RUN apt install -y cmake
RUN apt install -y nano
# RUN apt install -y tmux
RUN apt install -y git
RUN apt install -y unzip
# RUN apt update
# # RUN apt install -y qt5-default
# RUN apt install -y libqt5serialport5-dev
# RUN apt install -y build-essential
# RUN apt install -y libboost-system-dev
# # RUN apt install -y python-pip
# # RUN apt install -y python-dev
# # RUN pip install --upgrade pip
# # RUN pip install --upgrade virtualenv
# # RUN apt update
# RUN apt install -y pkg-config
# # RUN apt install -y liblz4-dev


# # Set non-interactive flag for configuring tzdata:
# ENV DEBIAN_FRONTEND=noninteractive

# # Install ROS Melodic (i.e. ROS 1)
# RUN apt update
# RUN apt install -y lsb-release
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# RUN apt update
# RUN apt install -y ros-melodic-desktop-full
# RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
# RUN rosdep init
# RUN rosdep update
# RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# RUN /bin/bash -c "source ~/.bashrc"


# # Install ROS Dashing (i.e. ROS 2)
# RUN apt update && apt -y install locales
# RUN locale-gen en_US en_US.UTF-8
# RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# RUN export LANG=en_US.UTF-8
# RUN apt update
# RUN sudo apt install -y curl
# RUN sudo apt install -y gnupg2
# RUN sudo apt install -y lsb-release
# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
# RUN apt update
# RUN apt install -y ros-dashing-desktop
# RUN apt install -y python3-argcomplete
# RUN apt update
# RUN apt install -y ros-dashing-ros1-bridge
# RUN apt install -y python3-colcon-common-extensions


## TMUX Configuration
#COPY tmux/.tmux.conf /root/
#COPY tmux/.tmux.conf.local /root/

# Copy MACE:
# COPY MACE_RELEASE_QT_LITE.tar.xz /root/


WORKDIR /
