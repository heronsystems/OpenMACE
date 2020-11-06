![MACE_Logo](https://github.com/heronsystems/OpenMACE/blob/master/docs/images/MACE_Logo.png) 
# OpenMACE
Open source version of the MACE architecture.

## Table of Contents
- [Description](#description)
- [Quick Start](#quick-start)
- [Doxygen Documentation](#documentation)
- [Acknowledgements](#acknowledgements)
- [LICENSE](#license)

# <a name="description"></a> Description
The Multi-Agent Collaborative Ecosystem (MACE) is a framework linking the communications, control, automation and human-machine interface components of a practical multi-vehicle system into a deployable package. MACE establishes the data management, scheduling, and monitoring required by a multi-vehicle robotic system.  Designed as a modular software architecture, MACE implements a core collaborative engine that exposes interfaces with other system components via APIs. This approach abstracts the details of the collaboration away from an individual system, allowing for rapid integration of third party components.

![MACEArchitecture](https://github.com/heronsystems/OpenMACE/blob/master/docs/images/maceArchitecture.png)

The modular architecture of MACE allows the software to be agnostic to the inner workings of a vehicle and its motion primitives. MACE implements a common interface between vehicle communications and the core software, and exposes a flexible API for vehicle developers to interface with other MACE enabled vehicles. Fundamentally, MACE is the backbone communications architecture that can facilitate a Swarm architecture (i.e. vehicle-to-vehicle communications, both 1-to-1 and 1-to-many). MACE provides a simple user interface, however MACE also implements a modular ground station API. Similar to the vehicle communications, this provides methods for third party developers to implement their own human-machine interfaces. Finally, MACE provides a resource and task allocation (RTA) API. This abstracts the tasking of a single vehicle within a swarm or the entire collective swarm to simple waypoints and commands. MACE can be applied to air vehicles, ground vehicles, surface vehicles, or a mixture of different vehicle types. The MACE architecture has been under active development since 2015 and is currently maturing alongside path planning and RTA research programs.

# <a name="qucik-start"></a> Quick Start
A more in depth installation guide is located on the MACE Wiki here: [Install](https://github.com/heronsystems/OpenMACE/wiki/Install-and-Run). For a quick start, You will need an ArduCopter simulation running, the MACE GUI running, and a simple MACE instance. 

First you have to clone MACE:

```
$ cd <desired>/<path>
$ git clone https://github.com/heronsystems/MACE
```
Once downloaded, you can build MACE using the [Qt Creator IDE](https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5) packaged with Qt. Simply navigate to `OpenMACE/src/` and open `src.pro`. MACE can be compiled using [MinGW](http://mingw.org/)--simply configure your project to build with MinGW in the creator Project menu. Run `QMake` and then `Build` on the project. 

Run MACE from the Qt Crator IDE and it will load the default XML file (located in `OpenMACE/MaceSetup_Configs/Default.xml`). Make sure to change the "ListenAddress" in the configuration file to the IP of your machine.

## <a name="ardupilot"></a> ArduPilot simulation
Follow the instructions [HERE](https://github.com/heronsystems/OpenMACE/wiki/ArduPilot-Simulation) to install ArduPilot SITL. To run a simulated vehicle, run the following in whatever directory you cloned your ardupilot repository in:

```
$ cd ardupilot/ArduCopter
$ sim_vehicle.py
```

## <a name="mace-gui"></a> MACE GUI
To run the MACE GUI, you will need [NodeJS](https://nodejs.org/en/) installed and configured as well as [Yarn](https://yarnpkg.com/en/docs/getting-started). Once configured, navigate to `OpenMACE/MACE_Frontend/` and run `yarn`. Once the installer installs the required Node packages, run the following command:

```
$ yarn start:app
```

If successful, the MACE GUI will run, and you should see a vehicle connected similar to below. Note: You may have to adjust the position of the map or the simulated vehicle.

![QuickStartGUI](https://github.com/heronsystems/OpenMACE/blob/master/docs/images/MACEGUIVehicle.png)


# <a name="documentation"></a> Doxygen Documentation

If any changes are made to the source code, please document them and re-run the Doxygen documentation generation. To do so, simply change into the `docs/` directory and run:

```
$ cd docs/
$ doxygen Doxyfile_default
```

There are also Bootstrap and Material design templates for styling the pages (`Doxyfile_bootstrap` and `Doxygile_material`, respectively). However, they are not as full-featured as the default documentation setup. If Doxygen is not installed, you can install on Ubuntu using

`sudo apt-get install doxygen`

For Windows users, you can try to use the [Doxywizard](https://www.stack.nl/~dimitri/doxygen/manual/doxywizard_usage.html). However, it is recommended to generate documentation using Linux.

# <a name="acknowledgements"></a> Acknowledgments
Heron Systems would like to akcnowledge the following individuals for their contributions on this software:
- NASA Langley Research Center
  - The MACE architecture was developed under a Phase 1/Phase 2 NASA SBIR program (Contract #: NNX16CL24C)
- Dr. Derek Paley, University of Marlyand, College Park
  - Dr. Paley provided subject matter expertise and algorithm development pertaining to resource and task allocations. Iterations of the algorithms developed with Dr. Paley's guidance were demonstrated both in simulation and onboard COTS platforms

# <a name="license"></a> LICENSE
This project is under the [GNU GPL v3 License](https://github.com/heronsystems/OpenMACE/blob/master/LICENSE)
