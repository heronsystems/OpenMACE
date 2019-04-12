# Electron GUI README

This is the GUI for the ground side of the MACE software suite. The goal of this GUI is to link a user interface to the MACE modules/libraries for display and high-level control of multiple vehicles. 

## Install
To build the GUI from source, you will need [Yarn](https://yarnpkg.com/en/docs/getting-started) installed.

Open a new console and change directories into the `MACE/ElectronGUI` directory. Install Node dependencies and build the React project:

```bash
yarn install
yarn run build:prod
```
If you delete the node_modules folder for whatever reason, re-run the above commands.

Now start the GUI:

In one terminal (Windows), run the watcher and the GUI
```bash
yarn run launch
```

For Linux, open two terminals and run:
```bash
**Terminal 1**
yarn run watch

**Terminal 2**
yarn run start
```

This will launch the GUI and the watcher. Give the GUI a minute to load as the watcher boots up. If any changes are made to the React project, the watcher should recompile the project and reload the GUI.

**NOTE:** In the past, we have seen errors resembling the following 
```
events.js:137
  throw er; // Unhandled 'error' event
  
Error: listen EACCES 127.0.0.1:8080
```
If this happens, re-build the GUI and relaunch with a different port number:
```
yarn run build:prod --nasa_gui:port=<newport>
yarn run launch --nasa_gui:port=<newport>
```
(Replace `<newport>` with the desired port number)

Alternatively, you can also edit the config in the `pacakge.json` file. Simply change the port number in that file and run the build and launch commands without any additional arguments.

## Prebuilt Binaries
If you do not want to build from source, there are prebuilt binaries included in the latest release version in a zipped file (`PrebuiltBinaries.zip`). This zipped file contains binary files for several architectures. Simply unzip the file and find your architecture and run the executable in the corresponding directory to start the MACE GUI. The following architectures are supported:

- Win32-x64
- Win32-ia32
- Linux-x64
- Linux-ia32
- Linux-armv7l

Note that this is a very large file, as Electron packages everything with the executables, including node_packages.

## Usage
### Selecting a vehicle
Many actions in the GUI require a vehicle ID to be set. You can select a vehicle by doing any of the following: 
- Click on the vehicle icon
- Click on the home icon
- Click on the Vehicle HUD on the right-hand side of the screen. 
  - This will open a "Set Home Location" dialog where you can set that particular vehicle's home location. If you do not wish to set the home location, simply close the dialog (by clicking off of it or hitting cancel) and nothing will be sent to the vehicle.
- With multiple vehicles connected, a dropdown appears next to the vehicle commands. Use that to select the vehicle ID

