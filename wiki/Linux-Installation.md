# Table of Contents
- [Prerequisites](#prereqs)
- [Build Steps](#build)
  - [DigiMesh Library](#digimesh)
    - [Build](#digimesh-build)
    - [Environment Variables](#digimesh-env-vars)
  - [MACE](#mace)
    - [Tools libraries](#installing-tools)
    - [Environment Variables](#mace-env-vars)
    - [Build](#mace-build)
  - [MACE GUI](#gui)
- [(Optional) ROS Setup](#ros-setup)
- [Running MACE](#run-mace)

# <a name="prereqs"></a> Prerequisites
Prior to building on Linux, you should make sure that your distribution is up to date. Run `sudo apt-get update && sudo apt-get upgrade` prior to running anything of the following steps.

## Build tools
To clone the repository, you will need to install [git command line interface](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git). Once installed, test that the installation was successful by typing `git --version` in a terminal window.

To install various third party libraries needed for MACE, you will also need to install [CMake](https://cmake.org/install/). Once installed, test that the installation was successful by typing `cmake --version` in a terminal window.

## MACE software prerequisites
Before compiling MACE from source, you will need a few things. First, you will need Qt ([Download installer from HERE](https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5)) and the Qt Creator IDE that comes with it. MACE currently depends on minimal libraries from Qt, however the easiest way to install MACE is still to use the Qt Creator IDE. Download the default installer and follow the steps. When you get the the components screen, make sure you select `gcc` under the Qt version you prefer (other build options such as Android build components are not required). For most users, the latest version of Qt is sufficient. The screenshot below shows an example of the components menu--note that the version numbers of Qt and gcc may not be up to date, this is just an example:

![QtInstaller_Linux](https://github.com/heronsystems/MACE/blob/master/docs/images/QtInstaller_Linux.png)

If you choose to install Qt using a method other than the online installer, you may need to install other build tools. However, this installation has not been tested without using the default Qt installation steps.

## MACE User Interface
If you wish to use the packaged user interface, you will need to install [NodeJS](https://nodejs.org/en/) for your platform as well as [Yarn](https://yarnpkg.com/en/docs/getting-started). For NodeJS, download the recommended installer and follow the default steps. Do the same for Yarn.

## ArduPilot SITL Simulation
If you wish to use a MAVLink simulated vehicle, the ArduPilot simulation has been used extensively with MACE. To install, follow the instructions [HERE](https://github.com/heronsystems/OpenMACE/wiki/ArduPilot-Simulation).

# <a name="build"></a> Build Steps
## <a name="digimesh"></a> Digimesh Library
Clone the Digimesh library distributed by Heron Systems by navigating to wherever you wish to clone the repository. An example would be /_code. In a command prompt:
```
$ cd /_code
$ git clone https://github.com/heronsystems/MACEDigiWrapper
```
In the directory where the wrapper was cloned, create three new directories: `MACEDigiWrapper/lib`, `MACEDigiWrapper/bin`, and `MACEDigiWrapper/include`.


### <a name="digimesh-build"></a> Building the Digimesh Wrapper
There are two options when building the Digimesh wrapper. Follow the instructions for your preferred method:

- [Qt Creator IDE build](#digimesh-qt-build)
- [Command line build](#digimesh-command-line-build)

#### <a name="digimesh-qt-build"></a> Qt Creator IDE build
Using the Qt Creator IDE, open the `MACEDigiWrapper.pro` file, making sure to configure the project using a `MinGW` enabled kit. Go to the `Projects` tab and select `Build`.

![Qt_ProjectsTab](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_ProjectsTab.png)

Add a build step using the dropdown and select `Make`.

![Qt_MakeDropdown](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_MakeDropdown.png)

In the `Make arguments:` box, add `install`. Generated headers and libraries should install to the `/include` and `/lib` directories off the root of the project made prior to building.

![Qt_MakeInstall](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_MakeInstall.png)

#### <a name="digimesh-command-line-build"></a> Command line build
Navigate to `/MACEDigiWrapper` and use `qmake` to generate Makefiles. Then `make` the libraries and install header files with the following commands:
```
$ cd /_code/MACEDigiWrapper
$ qmake
$ make
$ make install
```

To make library installs easier, we first want to add `*.conf` file that points to `MACEDigiWrapper/lib` directories. In the example below, `MACEDigiWrapper` was cloned in the `/_code` directory. Make sure to replace this with the appropriate path. This will allow us to "install" MACE libraries in order to run MACE via the command line.
```
$ echo "/_code/MACEDigiWrapper/lib" > /etc/ld.so.conf.d/MACEDigiWrapper.conf
```
After building any changes in the `MACEDigiWrapper` libraries, run the following command:
```
$ sudo ldconfig
```


### <a name="digimesh-env-vars"></a> Digimesh Environment Variables
To build MACE later, you will need to set environment variables for the MACEDigiWrapper. Open the `~/.bashrc` file with your preferred text editor and add the following lines to the end of the file:

```
export MACE_DIGIMESH_WRAPPER="/_code/MACEDigiWrapper"
export PATH=$PATH:/home/puffin/_code/MACEDigiWrapper/lib
```
Make sure to change the paths to where you cloned MACE (i.e. replace `/home/puffin/_code` with your directory structure).

Make sure to run `source ~/.bashrc` to have the changes take effect. This edit to the `.bashrc` file will have the `MACE_DIGIMESH_WRAPPER` set in every terminal and add the library directory to the path. The above path (i.e. `/_code/MACEDigiWrapper`) is an example, however this path should reflect wherever the root directory of the MACEDigiWrapper repository is. The `MACE_DIGIMESH_WRAPPER` environment variable should be set such that `%MACE_DIGIMESH_WRAPPER%/include` and `%MACE_DIGIMESH_WRAPPER%/lib` resolves to the appropriate directories.

***

## <a name="mace"></a> MACE
Clone the MACE repository distributed by Heron Systems by navigating to wherever you wish to clone the repository. An example would be `/_code`. In a command prompt:
```
$ cd /_code
$ git clone https://github.com/heronsystems/MACE
```
### <a name="mace-env-vars"></a> MACE Environment variables
Similar to the MACEDigiWrapper, we must add an environment variable to build MACE (see the [Digimesh Environment Variables](#digimesh-env-vars) section above). Add an environment variable `MACE_ROOT` set to that root such that `%MACE_ROOT%/include` and `%MACE_ROOT%/lib` resolves to the appropriate directories. Add the following lines to your `~/.bashrc` file:

```
export MACE_ROOT=/home/puffin/_code/MACE
export PATH=$PATH:/home/puffin/_code/MACE/lib
```

Make sure to change the paths to where you cloned MACE (i.e. replace `/home/puffin/_code` with your directory structure). Also, run `source ~/.bashrc` for the changes to take effect.

### <a name="installing-tools"></a> Tools libraries
Prior to building MACE, there are third party Tools libraries that must be installed. Follow the instructions on the wiki to install each library ([Installing Tools Libraries](https://github.com/heronsystems/OpenMACE/wiki/Installing-Tools-Libraries));


### <a name="mace-build"></a> Building the MACE application
Similar to the `MACEDigiWrapper` build steps above, you will need to add `MACE/bin`, `MACE/include`, and `MACE/lib` directories.

There are two options when building the MACE application. Follow the instructions for your preferred method:

- [Qt Creator IDE build](#mace-qt-build)
- [Command line build](#mace-command-line-build)

#### <a name="mace-qt-build"></a> Building with Qt Creator IDE
After making those directories, go to the `Projects` tab and select `Build`. Add a build step using the dropdown and select `Make`. In the `Make arguments:` box, add `install`. Generated headers and libraries should install to the `/include` and `/lib` directories off the root of the project made prior to building (See the screenshots above in the [Build steps -> Digimesh Library](#digimesh-qt-build) section for more detail).

Using the Qt Creator IDE, open the `MACE/src.pro` file, making sure to configure the project using a `gcc` enabled kit. Next, right click on the top level `src` directory and run `QMake`. Once this finishes, right click and run `Build` on the entire project. Note that this may take a while.

![Qt_RightClick](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_RightClick.png)


#### <a name="mace-command-line-build"></a> Building with Command Line
Next, change directory into the MACE root directory and build the project:

```
$ cd /_code/MACE
$ mkdir build
$ cd build
$ qmake ../src/src.pro
$ make
$ make install
```

To make library installs easier, we first want to add a `*.conf` file that points to `MACE/lib` directories. In the example below, `MACE` was cloned in the `/_code` directory. Make sure to replace this with the appropriate path. This will allow us to "install" MACE libraries in order to run MACE via the command line.
```
$ echo "/_code/MACE/lib" > /etc/ld.so.conf.d/MACE.conf
```
After building any changes in the `MACE` libraries, run the following command:
```
$ sudo ldconfig
```

## <a name="gui"></a> MACE GUI
To run the MACE GUI, you will need [NodeJS](https://nodejs.org/en/) installed and configured as well as [Yarn](https://yarnpkg.com/en/docs/getting-started). Once configured, navigate to `MACE/ElectronGUI/` and run `yarn install`. Once the installer installs the required Node packages, run the following commands.

Open a terminal and run:
```
$ yarn run watch
```
In a second terminal, run:
```
$ yarn run start
```

If successful, the MACE GUI will run, and you should see a blank map similar to the one below. Note that the `start` command above may take a few moments to actually start the GUI while it waits for the `watch` command to finish, and you may see a white screen while the command finishes.

![BlankGUI](https://github.com/heronsystems/MACE/blob/master/docs/images/blankGUI.png)

# <a name="ros-setup"></a> (Optional) ROS Setup
ROS can be used to simulate worlds and sensors. Steps to set up ROS and configure various vehicles can be found in the [ROS Setup](https://github.com/heronsystems/OpenMACE/wiki/ROS-MACE-Setup) wiki page.


# <a name="run-mace"></a> MACE Living and Breathing test
To test your installation, you can run a simple test with a simulated vehicle connected to a MACE instance, which can be displayed and controlled via the MACE GUI. This test is outlined in the [MACE Living and Breathing Test](https://github.com/heronsystems/OpenMACE/wiki/MACE-Living-and-Breathing-Test) wiki page.
