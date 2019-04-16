# Table of Contents
- [Prerequisites](#prereqs)
- [Build Steps](#build)
  - [DigiMesh Library](#digimesh)
    - [Environment Variables](#digimesh-env-vars)
  - [MACE](#mace)
    - [Installing Tools Libraries](#installing-tools)
    - [Build](#mace-build)
  - [MACE GUI](#gui)
- [Running MACE](#run-mace)

# <a name="prereqs"></a> Prerequisites
## Build tools
To clone the repository, you will need to install [git command line interface](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git). Once installed, test that the installation was successful by typing `git --version` in a terminal window.

To install various third party libraries needed for MACE, you will also need to install [CMake](https://cmake.org/install/). Once installed, test that the installation was successful by typing `cmake --version` in a terminal window.

## MACE software prerequisites
Before compiling MACE from source, you will need a few things. First, you will need Qt ([Download installer from HERE](https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5)) and the Qt Creator IDE that comes with it. MACE currently depends on minimal libraries from Qt, however the easiest way to install MACE is still to use the Qt Creator IDE. Download the default installer and follow the steps. When you get the the components screen, make sure you select MinGW under the Qt version you prefer (other build options such as Android build components are not required). For most users, the latest version of Qt is sufficient. The screenshot below shows an example of the components menu--note that the version numbers of Qt and MinGW may not be up to date, this is just an example:

![QtInstaller](https://github.com/heronsystems/MACE/blob/master/docs/images/QtInstaller.png)

If you choose to install Qt using a method other than the online installer, you may need to install [MinGW](http://mingw.org/)manually. To install MinGW, follow the steps on our wiki page [HERE](https://github.com/heronsystems/OpenMACE/wiki/Installing-MinGW-on-Windows).

## MACE User Interface
If you wish to use the packaged user interface, you will need to install [NodeJS](https://nodejs.org/en/) for your platform as well as [Yarn](https://yarnpkg.com/en/docs/getting-started). For NodeJS, download the recommended installer and follow the default steps. Do the same for Yarn.

## ArduPilot SITL Simulation
If you wish to use a MAVLink simulated vehicle, the ArduPilot simulation has been used extensively with MACE. To install, follow the instructions [HERE](https://github.com/heronsystems/OpenMACE/wiki/ArduPilot-Simulation).

# <a name="build"></a> Build Steps
## <a name="digimesh"></a> Digimesh Library
Clone the Digimesh library distributed by Heron Systems by navigating to wherever you wish to clone the repository. An example would be C:/Code. In a command prompt:
```
$ cd C:/Code/
$ git clone https://github.com/heronsystems/MACEDigiWrapper
```
In the directory where the wrapper was cloned, create three new directories: `MACEDigiWrapper/lib`, `MACEDigiWrapper/bin`, and `MACEDigiWrapper/include`.

Using the Qt Creator IDE, open the `MACEDigiWrapper.pro` file, making sure to configure the project using a `MinGW` enabled kit. Go to the `Projects` tab and select `Build`.

![Qt_ProjectsTab](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_ProjectsTab.png)

Add a build step using the dropdown and select `Make`.

![Qt_MakeDropdown](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_MakeDropdown.png)

In the `Make arguments:` box, add `install`. Generated headers and libraries should install to the `/include` and `/lib` directories off the root of the project made prior to building.

![Qt_MakeInstall](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_MakeInstall.png)

### <a name="digimesh-env-vars"></a> Digimesh Environment Variables
To build MACE, you will need to set environment variables for the MACEDigiWrapper. Open the Windows Control Panel and search for "Environment Variables." Select `Edit the system environment variables` and select "Environment Variables..." You should see the "Environment Variables" window appear:

![EnvironmentVariables](https://github.com/heronsystems/MACE/blob/master/docs/images/EnvironmentVariables.png)

Select "New..." and add an environment variable with the name `MACE_DIGIMESH_WRAPPER` and the value set to the root directory such that `%MACE_DIGIMESH_WRAPPER%/include` and `%MACE_DIGIMESH_WRAPPER%/lib` resolves to the appropriate directories. If the library was cloned into `C:/Code`, the variable value would be `C:/Code/MACEDigiWrapper`.

**Command line execution prerequisites:**

If you eventually wish to run MACE from the command line, you will need to add `%MACE_DIGIMESH_WRAPPER%\lib` to the `PATH` environment variable. Under `System variables`, select `Path` and then click `Edit`. In the dialog that appears, select `New` and add `%MACE_DIGIMESH_WRAPPER%\lib`, then press `OK`.

***

## <a name="mace"></a> MACE
Clone the MACE repository distributed by Heron Systems by navigating to wherever you wish to clone the repository. An example would be C:/Code. In a command prompt:
```
$ cd C:/Code
$ git clone https://github.com/heronsystems/MACE
```
### <a name="installing-tools"></a> Tools libraries
Prior to building MACE, there are third party Tools libraries that must be installed. Follow the instructions on the wiki to install each library ([Installing Tools Libraries](https://github.com/heronsystems/OpenMACE/wiki/Installing-Tools-Libraries));

### <a name="mace-env-vars"></a> MACE Environment Variables
Similar to the MACEDigiWrapper, we must add an environment variable to build MACE (see the [Digimesh Environment Variables](#digimesh-env-vars) section above). Add an environment variable `MACE_ROOT` set to that root such that `%MACE_ROOT%/include` and `%MACE_ROOT%/lib` resolves to the appropriate directories.

**Command line execution prerequisites:**

If you eventually wish to run MACE from the command line, you will need to add `%MACE_ROOT%\lib` and `%MACE_ROOT%\bin` to the `PATH` environment variable. Under `System variables`, select `Path` and then click `Edit`. In the dialog that appears, select `New` and add `%MACE_ROOT%\lib`. Do the same for `%MACE_ROOT%\bin`, then press `OK`.

Since MACE is built using some Qt libraries, we should also add Qt to the Path variable for command-line execution. Using the same process as above, add the path to your Qt MinGW libraries. As an example, if Qt was installed using the default methods, then you would add the following to your PATH variable (substituting the Qt version number for the one you installed):
```
C:\Qt\5.7\mingw53_32\lib
C:\Qt\5.7\mingw53_32\bin
```

Finally, MACE relies on Octomap (in the `tools` directory) for path planning functionalities. Add the octomap library path to the `PATH` variable using the same process as above. The path to Octomap is `%MACE_ROOT%\tools\octomap\bin`, if the default installation was followed for MACE Tools libraries.

### <a name="mace-build"></a> Building the MACE application
Similar to the `MACEDigiWrapper` build steps above, you will need to add `MACE/bin`, `MACE/include`, and `MACE/lib` directories.

After making those directories, go to the `Projects` tab and select `Build`. Add a build step using the dropdown and select `Make`. In the `Make arguments:` box, add `install`. Generated headers and libraries should install to the `/include` and `/lib` directories off the root of the project made prior to building (See the screenshots above in the [Build steps -> Digimesh Library](#digimesh-library) section for more detail).

Using the Qt Creator IDE, open the `MACE/src.pro` file, making sure to configure the project using a `MinGW` enabled kit. Next, right click on the top level `src` directory and run `QMake`. Once this finishes, right click and run `Build` on the entire project. Note that this may take a while.

![Qt_RightClick](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_RightClick.png)

***

## <a name="gu"></a> MACE GUI
To run the MACE GUI, you will need [NodeJS](https://nodejs.org/en/) installed and configured as well as [Yarn](https://yarnpkg.com/en/docs/getting-started). Once configured, navigate to `MACE/ElectronGUI/` and run `yarn install`. Once the installer installs the required Node packages, run the following commands.

Open a command prompt and run:
```
$ yarn run launch
```
If successful, the MACE GUI will run, and you should see a blank map similar to the one below. Note that the `launch` command above may take a few moments to actually start the GUI, and you may see a white screen while the command finishes.

![BlankGUI](https://github.com/heronsystems/MACE/blob/master/docs/images/blankGUI.png)


# <a name="run-mace"></a> MACE Living and Breathing test
To test your installation, you can run a simple test with a simulated vehicle connected to a MACE instance, which can be displayed and controlled via the MACE GUI. This test is outlined in the [MACE Living and Breathing Test](https://github.com/heronsystems/OpenMACE/wiki/MACE-Living-and-Breathing-Test) wiki page.
