UMD_TaskGeneration MATLAB Code: README                      
Last Modified: 12 Dec. 2018

Overview:

This folder contains code for simulating a swarm of quad copters operating in 
an urban environment. The code aims to generate tasks that balance the swarms 
priorities of:
 - Mapping an unknown environment and building an occupancy graph on-the-fly.
 - Search for targets using a log-likelihood ratio detection and tracking (LRDT)
   framework.

Content Description:

  main_taskGeneration.m   The top-level function for starting a simulation
  main_F3CheckoutFlight.m The top-level function for conducting basic checkout flights 
                          at UMD's Fearless Flight Facility (F3).
  MonteCarloEngine.m      Used for conducting monte-carlo performance analysis
  ./subroutines           The majority of the UMD-developed code resides here 
  ./external              Third-party packages that are used in this project
  ./data                  Folder for storing static data (e.g., map files)
  ./launchMACE.sh         A shell script for launching the MACE environment
  ./killMACE.sh           A shell script for ending all processes related to MACE

Requirements:
  - Ubuntu 16.04 LTS Operating System
  - MATLAB Version: 9.4.0.813654 (R2018a)
  - MATLAB Robotics System Toolbox Version 2.0 (R2018a)
  - MATLAB Image Processing Toolbox Version 10.2 (R2018a)
  - MACE (including all dependencies: ROS, Ardupilot, etc.)

Environment Variables:
  - $MACE_ROOT        Pointing to the root mace directory
  - $ARDUIPILOT_ROOT  Pointing to the root ardupilot directory 
  - Run updatePath.m to make sure all of the subroutines/data/external
    files are in your MATLAB path   

Running the code (MATLAB):
  - This code can run in MATLAB-only mode by setting:
    runParams.type = 'matlab'; 
    in the main_taskGeneration.m 
  - There are several parameters (e.g., number of agents, map, behavior types, 
    communication topology, penalty/reward calculation) 
    that can be modified by changing the appropriate values in 
    on of the "loadParams" files. 
  - There currently are several loadParams files that allow the user to load
    different environments.
      loadParams_cityBlocks() : loads an idealized 4 city block environment (fully connected)
      loadParams_Randals() : loads a realistic 4 city block environment based on Randalls Island, NY
      loadParams_RandalsAtF3() : loads a subsection of Randals Island map that 
        is scaled and rotated to fit into F3 test arena
  - Run updatePath.m to make sure all of the subroutines/data/external
    files are in your path
  - Run main_taskGeneration.m
  - Once a simulation is complete, press any key to playback the video.
      - Note numerous video options are available and the correct option must 
        be selected depending on the particular behavior/configuration. For example,
        depending on the configuration of the cityBlocks environment one of the
        following movies can be selected:
        % movie_cityBlocks_lrdt
        % movie_cityBlocks_nodeDensity
        % movie_cityBlocks_mutualInfoPriors
        % movie_cityBlocks_mutualInfoWpts

Running the code (MACE):
  - This code can also use MACE to simulate vehicle dynamics/control. This mode
    requires setting:
    runParams.type = 'mace';
  - Assuming MACE and MATLAB are running on the same computer, you must modify 
    the IP address in two locations:
      In main_taskGeneration.m, change: ROS_MACE.ip ='10.104.193.156'; 
      In launch.sh, change: IP_ADDRESS=10.104.193.156
  - To launch the MACE environment with four vehicles and the GUI:
    Navigate to the UMD_TaskGeneration folder in the terminal and run:
    $ ./launchMACE.sh -n 4 -g
  - Run updatePath.m to make sure all of the subroutines/data/external
    files are in your path
  - Run main_taskGeneration.m
  - Wait until GPS origin is set before sending the datum
  - Once a simulation is complete, press any key to playback the video.

For any questions regarding how to run this code please contact the UMD team:
  Artur Wolek (wolek@umd.edu)
  Sheng Cheng (cheng@umd.edu)
  Derek Paley (dpaley@umd.edu)