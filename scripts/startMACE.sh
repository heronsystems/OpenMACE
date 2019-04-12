#! /bin/bash

sleep 7

export LD_LIBRARY_PATH=/usr/local/Qt-5.7.0/lib:$LD_LIBRARY_PATH
export PATH=$PATH:/usr/local/Qt-5.7.0/lib
export PATH=$PATH:/usr/local/Qt-5.7.0/bin
export PATH=$PATH:$HOME/_code/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
source /opt/ros/kinetic/setup.bash
source /home/odroid/_code/MACE_DecentralizedRTA/catkin_sim_environment/devel/setup.bash
#mate-terminal -e "read -n 1 -s -r -p 'Press any key to continue'"
mate-terminal -e "/home/odroid/_code/MACE_DecentralizedRTA/bin/MACE"

#xterm -e "read -n 1 -s -r -p 'Press any key to continue'"
#xterm -e "/home/odroid/_code/MACE_DecentralizedRTA/bin/MACE;bash"
#xterm -e "echo 'TEST'"
