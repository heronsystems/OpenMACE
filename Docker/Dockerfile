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
#Download base image ubuntu 20.04
FROM ubuntu:20.04

# Update Ubuntu Software repository
RUN apt update
## START CHANGING STUFF

# Install tools here, (recommended to use multiple lines so they don't have to all reinstall if you change one)
RUN apt install -y cmake
RUN apt install -y nano
RUN apt install -y tmux
RUN apt install -y git
RUN apt install -y unzip
RUN apt update
# RUN apt install -y qt5-default
RUN apt install -y libqt5serialport5-dev
RUN apt install -y build-essential
RUN apt install -y libboost-system-dev
RUN apt install -y pkg-config
# RUN apt install -y liblz4-dev

RUN git clone https://github.com/heronsystems/OpenMACE.git

WORKDIR /
