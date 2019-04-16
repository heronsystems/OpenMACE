# Table of Contents
- [Running MACE](#mace)
- [Running the MACE GUI](#gui)
  - [Running with Qt Creator IDE](#mace-qt)
  - [Running from command line](#mace-command-line)
- [Running a simulated vehicle](#simulated-vehicle)
- [Troubleshooting](#troubleshooting)

# <a name="mace"></a> Prerequisites
This test assumes that you have installed MACE and the supporting libraries. If you have yet to do so, install MACE by following the steps outlined in either of the following pages:

- [Windows Installation](https://github.com/heronsystems/OpenMACE/wiki/Windows-Installation)
- [Linux Installation](https://github.com/heronsystems/OpenMACE/wiki/Linux-Installation)

# <a name="mace"></a> Running MACE
## <a name="mace-configuration-file"></a> MACE Configuration File
Before you run MACE, you need to make sure your configuration file is set up properly. By default, MACE loads the `Default.xml` file, located in `MACE/MaceSetup_Configs/Default.xml`. For this test, we will edit this file to connect to a simulated vehicle and the MACE GUI. Open the `Default.xml` configuration file in any text editor and change the "ListenAddress" and "GUIHostAddress" in the configuration file to the IP of your machine. An example XML file is shown below:

```xml
<?xml version="1.0" encoding="utf-8"?>
<ModuleConfigurations MaceInstance="1">
  <Module Class="VehicleComms" Type="Ardupilot">
    <Parameter Name="ProtocolParameters">
      <Parameter Name="Name">Mavlink</Parameter>
      <Parameter Name="Version">V2</Parameter>
    </Parameter>
	<Parameter Name="UDPParameters">
      <Parameter Name="ListenAddress">192.168.1.21</Parameter>
      <Parameter Name="ListenPortNumber">14551</Parameter>
    </Parameter>
    <Parameter Name="ModuleParameters">
      <Parameter Name="AirborneInstance">false</Parameter>
    </Parameter>
  </Module>

  <Module Class="GroundStation" Type="NASAPhase2">
	<Parameter Name="ID">1</Parameter>
	<Parameter Name="MACEComms">
		<Parameter Name="GUIHostAddress">192.168.1.21</Parameter>
		<Parameter Name="ListenPort">5678</Parameter>
		<Parameter Name="SendPort">1234</Parameter>
	</Parameter>
   </Module>
</ModuleConfigurations>
```

Note that you would want to change the `ListenAddress` and `GUIHostAddress` to the correct IP addresses. If you are running a simulated vehicle and the GUI both on your local machine, change both addresses to the IP address of your local machine.

With the configuration file setup, there are two ways to run the MACE application. For most users, the easiest way to run it is from the Qt Creator IDE. Follow the instructions in one of the following sections based on your preference:

- [Running with Qt Creator IDE](#mace-qt)
- [Running from command line](#mace-command-line)

## <a name="mace-qt"></a> Running with Qt Creator IDE
Open the Qt Creator IDE and load the `MACE/src.pro` file to load the MACE project. Open the `Projects` tab and select the `Run` option. In the `Command line arguments:` box, enter the relative path from the root `MACE` directory (i.e. where your `MACE_ROOT` environment variable points to) to where your desired XML configuration file is. If this box is left blank, MACE will load the configuration file `MACE/MaceSetup_Configs/Default.xml`. An example screenshot is shown below loading the `Default.xml` file.

![Qt_RunArguments](https://github.com/heronsystems/MACE/blob/master/docs/images/Qt_RunArguments.png)

Once you have added the correct path to your desired configuration file, click the green "Play" button in the bottom left of the GUI to run the MACE application. You should see a command prompt open up and show text similar to that shown below:

![MACE](https://github.com/heronsystems/MACE/blob/master/docs/images/MACE.png)


## <a name="mace-command-line"></a> Running from command line
Navigate to the `MACE/bin` directory and run the following command:

```
$ ./MACE MaceSetup_Configs/Default.xml
```
_Note that on Windows, the `./` is not necessary_

If the command executed correctly, you will see text similar to that shown below:

![MACE](https://github.com/heronsystems/MACE/blob/master/docs/images/MACE.png)


# <a name="gui"></a> Running the MACE GUI
The GUI loads a separate JSON configuration file located in the root MACE directory (`MACE/GUIConfig.json`). Make sure that the IP address for the `MACEComms` matches the IP address where the MACE instance is running. An example configuration is shown below for a connection with the MACE instance we started above:

```json
{
    "MACEComms": {
        "ipAddress": "192.168.1.21",
		"listenPortNumber": 1234,
		"sendPortNumber": 5678
    },
	"GUIInit": {
		"mapCenter": {
			"lat": 37.889246,
			"lng": -76.814084
		},
		"mapZoom": 17
	},
	"VehicleSettings": {
		"defaultTakeoffAlt": 10
	}
}
```
Again, make sure the above IP address is set to the IP address of your machine.

With the configuration file in place, run the MACE GUI:
On Windows, open a command prompt and run:
```
$ yarn run launch
```
 -- OR --

On Linux, open two separate terminals:
```
**Terminal 1:**
$ yarn run watch

*** Terminal 2:**
$ yarn run start
```

If successful, the MACE GUI will run, and you should see a blank map similar to the one below. Note that the `launch`/`start` command above may take a few moments to actually start the GUI while it waits for the watcher to start up, and you may see a white screen while the command finishes.

![BlankGUI](https://github.com/heronsystems/MACE/blob/master/docs/images/blankGUI.png)

If there is a connection between the MACE GUI and the MACE instance that was started above, you should see `No vehicles currently available` printing in the MACE terminal about every second (see screenshot). In the next section, we will start a simulated vehicle to display on the GUI and talk to the MACE application.

![NoVehicles](https://github.com/heronsystems/MACE/blob/master/docs/images/NoVehicles.png)


# <a name="simulated-vehicle"></a> Running a simulated vehicle
To run a simulated vehicle, open a terminal and navigate to wherever `ardupilot` was cloned. **NOTE: On Windows, you will have to do this in a Cygwin temrinal, NOT the regular Windows command prompt**.

Change into the `ArduCopter` directory and run the `sim_vehicle.py` script with a few arguments:
- `-I`: denotes the vehicle ID
- `--out=udp=`: specify an IP:PORT combination. The IP should be set to the "ListenAddress" IP in the MACE XML configuration file, and the PORT should be set to the "ListenPortNumber" in the same configuration file
- `--custom-location`: specify a location to start the vehicle at. Format is: `lat,lon,alt,heading`

```
$ cd ardupilot/ArduCopter
$ sim_vehicle.py -I 1 --console --out=udp:192.168.1.21:14551 --custom-location=37.890425,-76.811910,0,240
```
If you run multiple vehicles, the ID's set with the `-I` flag must be unique. In order to use this flag, make sure you have cloned the Heron Systems fork of the Ardupilot directory (see the note in the Installation section of the [ArduPilot Simulation](https://github.com/heronsystems/OpenMACE/wiki/ArduPilot-Simulation) wiki page).

If successful, you should see a terminal window and "console" window appear that resemble the screenshot below:
![ArduPilotSim](https://github.com/heronsystems/MACE/blob/master/docs/images/ArduPilotSim.png)

If the configurations were set up correctly, you should see output similar to the following in the MACE terminal:
![MACEConnectedVehicle](https://github.com/heronsystems/MACE/blob/master/docs/images/MACEConnectedVehicle.png)

You should also see a connected vehicle in the MACE GUI, similar to the screenshot below. Note that you most likely will not see a mission appear unless you have previously loaded the vehicle with a mission.
![MACEGUIVehicle](https://github.com/heronsystems/MACE/blob/master/docs/images/MACEGUIVehicle.png)

If everything was set up correctly and you see a vehicle on the MACE GUI, try executing a simple command to ensure two-way communications are enabled. "Select" the vehicle by either clicking on its icon on the map, or by clicking the "HUD" on the left hand side of the screen by the vehicle ID. Once selected, an `Arm` button will appear at the bottom of the GUI. Click that button and you should see a message appear in the vehicle "HUD" that says "Arming motors." Once armed, the vehicle icon in the "HUD" should turn green for a few seconds until the motors disarm themselves for inactivity.


# <a name="troubleshooting"></a> Troubleshooting
A common issue when running MACE with a vehicle and a GUI is not being able to see the vehicle on the GUI. There are a few common cases:
1. If MACE has connection with the GUI, but NOT the vehicle, then MACE should be printing out "No vehicles currently available."
2. If MACE has connection with the Vehicle, but NOT the GUI, then MACE should be printing things like "Failed to write Vehicle Heartbeat" (among other things).
3. If MACE has connection with neither the GUI nor the Vehicle, then you shouldn't see anything printing in the MACE terminal

In cases #2 and #3, the GUI console (which you can toggle by pressing Ctrl+Shift+I, or by going to View->Toggle Developer Tools) should be outputting "Socket Error: connect ECONNREFUSED <IP>:<PORT>". This basically means the GUI can't see MACE.

***

For Case #1, first check that your MACE configuration file (default is `MACE/MaceSetup_Configs/Default.xml`) is pointing to the correct vehicle. For simulated vehicles, check the `UDPParameters` for the correct IP and port combination. This IP and port combination should match the IP and port that is being output from the ArduPilot simulation. For example, if the simulated vehicle was run via the following command:

```
$ sim_vehicle.py --console -m --out=udp:127.0.0.1:14552 -I 2
```

then the `VehicleComms` portion of the MACE configuration file would look like:

```xml
  <Module Class="VehicleComms" Type="Ardupilot">
    <Parameter Name="ProtocolParameters">
      <Parameter Name="Name">Mavlink</Parameter>
      <Parameter Name="Version">V2</Parameter>
    </Parameter>
	<Parameter Name="UDPParameters">
      <Parameter Name="ListenAddress">127.0.0.1</Parameter>
      <Parameter Name="ListenPortNumber">14552</Parameter>
    </Parameter>
    <Parameter Name="ModuleParameters">
      <Parameter Name="AirborneInstance">false</Parameter>
    </Parameter>
  </Module>
```
If this does not resolve the issue, check that you are outputting the correct IP and port combination on the simulator. Run the command `output` in the ardupilot terminal and make sure your IP:Port combination is present. If it is not present, you can add it manually by running `output add <IP>:<PORT>`.

***

For Case #2, this typically means the IP and port combinations are incorrect for the MACE configuration file and/or the GUI configuration file. For a GUI on the same machine as the MACE instance, the `GroundStation` module portion of the MACE configuration file should look like:
```xml
    <Module Class="GroundStation" Type="NASAPhase2">
	<Parameter Name="ID">1</Parameter>
	<Parameter Name="MACEComms">
		<Parameter Name="GUIHostAddress">127.0.0.1</Parameter>
		<Parameter Name="ListenPort">5678</Parameter>
		<Parameter Name="SendPort">1234</Parameter>
	</Parameter>
   </Module>
```
Then on the GUI side, open the `GUIConfig.json` file in the root directory of MACE. This configuration should mimic the MACE XML file, and in the case of a local setup matching the above MACE Ground station setup, the file should look like:
```json
{
    "MACEComms": {
        "ipAddress": "127.0.0.1",
		"listenPortNumber": 1234,
		"sendPortNumber": 5678
    },
	"GUIInit": {
		"mapCenter": {
			"lat": 37.889246,
			"lng": -76.814084
		},
		"mapZoom": 17
	},
	"VehicleSettings": {
		"defaultTakeoffAlt": 10
	}
}
```
Note that the send and listen ports of the GUI configuration are opposite of those in the MACE XML file. i.e. the send port for MACE should be the listen port for the GUI.

***

For Case #3, you will have to check the fixes for both Case #1 and Case #2 as the issue is likely some combination of both.
