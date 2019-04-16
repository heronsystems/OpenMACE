# Table of Contents
- [Description](#description)
- [Docker Install](#docker-install)
  - [Windows Troubleshooting](#windows-troubleshooting)
- [Building and Running MACE with Docker](#build-run-mace-docker)
  - [Build](#build-mace-docker)
  - [Run](#run-mace-docker)
    - [Run the Container](#run-mace-container-docker)
    - [Run the MACE Application](#run-mace-app-docker)
- [Building and Running ArduPilot SITL with Docker](#build-run-sitl-docker)
  - [Build](#build-sitl-docker)
  - [Run](#run-sitl-docker)
    - [Run the Container](#run-sitl-container-docker)
    - [Run the ArduPilot SITL instance(s)](#run-sitl-app-docker)
- [Sharing files between Host Machine and Docker Container](#sharing)
- [Simulate MACE with Docker Containers](#mace-sim-docker)
  - [Start MACE Container](#sim-mace-conatiner)
  - [Start SITL Container](#sim-sitl-container)
  - [Modify configuration files](#modify-configs)
    - [MACE Configuration](#mace-config)
    - [SITL Configuration](#sitl-config)
  - [Start the Simulation](#start-sim)
- [TODO](#todo)

# <a name="description"></a> Description
Docker allows developers to keep a consistent development environment across platforms. Using "containers", we can set up an environment via build files that build a consistent environment to deploy any application. Containers remove the need to have a native OS or full-up virtual machine to deploy applications. They also help keep development environments consistent across developers to quickly discover build or run issues with new code. Docker runs on Windows, Linux, and Mac.

A more in-depth description of what Docker provides can be found at the [Docker](https://www.docker.com/what-docker) website.

# <a name="docker-install"></a> Docker Install
To install Docker on your platform, use one of the following:
* [Windows](https://docs.docker.com/docker-for-windows/install/)
* [Linux](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [Mac](https://docs.docker.com/docker-for-mac/install/)
  - Note that the Mac Docker setup has not been verified for the MACE project

## <a name="windows-troubleshooting"></a> Windows Troubleshooting
Docker on Windows uses Hyper-V to create a virtual machine in the background to run containers. With certain
When installing Docker on Windows, there was an issue with certain display drivers, particularly those associated with the Intel(R) HD Graphics 4600 display adapter. After Docker is installed, it will ask to enable Hyper-V (if it isn't already enabled). This may cause your display to find one or more displays that are not actually connected, which will act as external monitors that cannot be seen. To fix this issue, the first workaround is to try updating your display adapter driver. Open the Device Manager and find your Display Adapter. Right click and select "Update Driver". You will most likely need to restart for the changes to take effect.

If updating the driver does not fix the issue, the next step is to try downgrading your driver to a compatible driver. A good tutorial for fixing this issue can be found [HERE](https://www.youtube.com/watch?v=dYfeOnFZegI&t=1s).

# <a name="build-run-mace-docker"></a> Building and Running MACE with Docker
_NOTE: Windows PowerShell has some issues displaying data in containers properly, so it is recommended to use the default Command Prompt_.

## <a name="build-mace-docker"></a> Build
To build MACE using Docker, simply navigate to the `MACE/Docker` directory and use the `Dockerfile_MACE` file to build the docker image. In a terminal/command prompt:

```
cd <path>/<to>/<MACE>/Docker
docker build -t ubunutu1604:mace -f ./Dockerfile_MACE
```
Replace `<path>/<to>/<MACE>` with your local directory path to the MACE root directory.

The above build command uses the `Dockerfile_MACE` file to build a container image. This particular image will be named `ubuntu1604:mace`, however it can be named anything.

## <a name="run-mace-docker"></a> Run
### <a name="run-mace-container-docker"></a> Run the container
Now that you have built a docker image, you can run a docker container using that image. The basic command to run a docker container is `docker run -it <image_name>`. In this case, however, we also need to expose a port on the container to the host machine for MACE communications. To do so, run the following:

```
docker run --name mace -p 5678:5678 -it ubuntu1604:mace
```

The “-p 5678:5678” option tells the docker container to forward the host’s port 5678 to the container's port 5678. If you change the MACE config away from the default, make sure the ground station ports and GUI ports match with any changed options and modify the above command accordingly. Note that the container will also be named `mace` via the option `--name`. This can also be changed to anything.

### <a name="run-mace-app-docker"></a> Run the MACE application
When the MACE container starts up, you should see that you are in the `/MACE/` directory. To make sure that MACE runs, simply run:

```
./bin/MACE
```

This will start MACE with the default configuration (located in the `MaceSetup_Configs/` directory). To get a MACE instance talking with a simulated vehicle, follow the steps below in the [Simulate MACE with Docker Containers](#mace-sim-docker) section.


# <a name="build-run-sitl-docker"></a> Building and Running ArduPilot SITL with Docker
_NOTE: Windows PowerShell has some issues displaying data in containers properly, so it is recommended to use the default Command Prompt_.

## <a name="build-sitl-docker"></a> Build
To build ArduPilot SITL using Docker, simply navigate to the `MACE/Docker` directory and use the `Dockerfile_SITL` file to build the docker image. In a terminal/command prompt:

```
cd <path>/<to>/<MACE>/Docker
docker build -t ubunutu1604:sitl -f ./Dockerfile_SITL
```
Replace `<path>/<to>/<MACE>` with your local directory path to the MACE root directory.

The above build command uses the `Dockerfile_SITL` file to build a container image. This particular image will be named `ubuntu1604:sitl`, however it can be named anything.

## <a name="run-sitl-docker"></a> Run
### <a name="run-sitl-container-docker"></a> Run the container
Now that you have built a docker image, you can run a docker container using that image. The basic command to run a docker container is `docker run -it <image_name>`. To run the SITL container, run the following:

```
docker run --name sitl –it ubuntu1604:sitl
```
Note that the container will be named `sitl` via the option `--name`. This can also be changed to anything.

### <a name="run-sitl-app-docker"></a> Run the ArduPilot SITL instance(s)
When the SITL container starts up, you should see that you are in the `/ardupilot/Tools/autotest/` directory. To make sure that the ArduPilot SITL runs, simply run:

```
python sim_multi_vehicle.py MACEConfigs/sample_1Vehicle.json
```

This will launch a `tmux` session with two windows--a blank bash shell and a SITL instance. If you click between the tabs (from the bottom tab bar), you should see the SITL instance boot up. To get a SITL vehicle talking with MACE, follow the steps below in the [Simulate MACE with Docker Containers](#mace-sim-docker) section.

# <a name="sharing"></a> Sharing files between Host Machine and Docker Container
With Docker, it is possible to share directories/files between the host machine and the docker container using “volumes”. For MACE, this is most useful for configuration files, as modifying files within the container can be cumbersome with “nano” or “vim” type text editors. If you wish to share a directory between machines, modify the “docker run …” command from the `Run` sections above to include the shared volume.

On the host machine, create a directory that you wish to share. Alternatively, you can also use existing directories. **HOWEVER**--if you use an existing directory on the container side, the directory on the container side will be overwritten with the contents of the host's shared directory.

For the MACE container, it may be useful to use the `MACE/MaceSetup_Configs` directory with configuration files already populated. Likewise, for the SITL container, it may be useful to use the `ardupilot/Tools/autotest/MACEConfigs` directory. Assuming the docker images have already been built according to the steps outlined above, we need to start each container making sure to point them to the correct directories to share. For example:

***

**_MACE_**
```
docker run --name mace -v MACE/MaceSetup_Configs:MACE/MaceSetup_Configs -p 5678:5678 -it ubuntu1604:mace
```
This will run a container named `mace` and share the `MACE/MaceSetup_Configs` directory with the container. The container will use `MACE/MaceSetup_Configs` on its local file system to sync with the shared directory on the Host. We will also forward port `5678` on the host to the same port on the container. This container will be run using the `ubuntu1604:mace` image.

**_SITL_**
```
docker run --name sitl -v ardupilot/Tools/autotest/MACEConfigs/:ardupilot/Tools/autotest/MACEConfigs –it ubuntu1604:sitl
```
This will run a container named `sitl` and share the `rdupilot/Tools/autotest/MACEConfigs/` directory with the container. The container will use `rdupilot/Tools/autotest/MACEConfigs/` on its local file system to sync with the shared directory on the Host. This container will be run using the `ubuntu1604:sitl` image.

***

If you have set up your shared volumes correctly, you should be able to edit files/create files/delete files on either the Host or container and the changes will be reflected across the other machine. This makes it easier to edit configuration files so that launching MACE or the SITL instances becomes less cumbersome.

# <a name="mace-sim-docker"></a> Simulate MACE with Docker Containers
The above sections walked through how to build and run the basic MACE and SITL containers. This section will detail further how to run a MACE test with Docker. In this section, we will:
1. Start a MACE container (with shared files)
2. Start a SITL container (with shared files)
3. Start the MACE GUI (on the Host machine)

The following steps assume that the MACE and SITL containers have been already built. We also assume the the MACE GUI has been installed on the Host machine. See the [MACE Install and Run Steps](https://github.com/heronsystems/OpenMACE/wiki/Install-and-Run#mace-build) to set up the GUI if it has not been previously set up.

## <a name="sim-start-mace"></a> Start MACE Container
We want to start the MACE container and share a local directory as before:
```
docker run --name mace -v MACE/MaceSetup_Configs:MACE/MaceSetup_Configs -p 5678:5678 -it ubuntu1604:mace
```
After the container starts up, we need to make sure our configuration file is set up properly. To do so, we need to determine the network settings of the container. At the prompt on the container, run: `ping host.docker.internal` and take note of the IP address. Stop its execution, and open another terminal/command prompt.

In the new window, run `docker ps -a` and take note of the container ID for the MACE container. Next, run `docker inspect <container_id>`, where the `<container_id>` is the first couple characters of the MACE container ID. The output of the `inspect` command can be very large, however we are only interested in the `NetworkSettings` portion of the output. Take note of the IP Address and the Gateway. They should be something like `172.17.0.1`. An example of the output of the `inspect` command is shown below:
```json
"Networks": {
    "bridge": {
        "IPAMConfig": null,
        "Links": null,
        "Aliases": null,
        "NetworkID": "<network_id>",
        "EndpointID": "<endpoint_id",
        "Gateway": "172.17.0.1",
        "IPAddress": "172.17.0.2",
        "IPPrefixLen": 16,
        "IPv6Gateway": "",
        "GlobalIPv6Address": "",
        "GlobalIPv6PrefixLen": 0,
        "MacAddress": "<mace_address>",
        "DriverOpts": null
    }
}
```

## <a name="sim-start-sitl"></a> Start SITL Container
Similar to the MACE container, we want to start the SITL container and share a local directory:
```
docker run --name sitl -v ardupilot/Tools/autotest/MACEConfigs/:ardupilot/Tools/autotest/MACEConfigs –it ubuntu1604:sitl
```
Similar to the MACE container, we need to make sure our configuration file is set up properly. To do so, we need to determine the network settings of the container. Run `docker ps -a` and take note of the container ID for the SITL container. Next, run `docker inspect <container_id>`, where the `<container_id>` is the first couple characters of the SITL container ID. Again, we are only interested in the `NetworkSettings` portion of the output. Take note of the IP Address and the Gateway. They should be something like `172.17.0.1`. An example of the output of the `inspect` command is shown above.

## <a name="modify-configs"></a> Modify configuration files
The containers started above were started with a volume shared between host and container that contains configuration files for both MACE and the ArduPilot SITL isntances. For MACE, we want an XML configuration file with one `VehicleComms` module and a `GroundStation` module, both with the correct network settings. For SITL, we want a JSON configuration file with one vehicle. For the next couple sections, the network configuration that was determined previously will be used. For this example, the network settings were as follows:

* MACE Container
  * **`host.docker.internal` IP address**: 192.168.65.2
  * **Container gateway**: 172.17.0.1
  * **Container IP**: 172.17.0.2
* SITL Container
  * **Container IP**: 172.17.0.3

### <a name="mace-config"></a> MACE Configuration
In the `MACE/MaceSetup_Configs` directory on the Host machine, open the `Default.xml` file. Any modifications to this file will be reflected to the container in the directory specified in the run command above. Modify the file to match the following, making sure to substitute your network settings where necessary:

```xml
<?xml version="1.0" encoding="utf-8"?>
<ModuleConfigurations>

  <Module Class="VehicleComms" Type="Ardupilot">
    <Parameter Name="ProtocolParameters">
      <Parameter Name="Name">Mavlink</Parameter>
      <Parameter Name="Version">V2</Parameter>
    </Parameter>
	<Parameter Name="UDPParameters">
      <Parameter Name="ListenAddress">172.17.0.3</Parameter>
      <Parameter Name="ListenPortNumber">14558</Parameter>
    </Parameter>
    <Parameter Name="ModuleParameters">
      <Parameter Name="AirborneInstance">false</Parameter>
    </Parameter>
  </Module>

  <Module Class="GroundStation" Type="NASAPhase2">
	<Parameter Name="ID">1</Parameter>
	<Parameter Name="MACEComms">
		<Parameter Name="GUIListenAddress">172,17.0.1</Parameter>
		<Parameter Name="GUISendAddress">192.168.65.2</Parameter>
		<Parameter Name="ListenPort">5678</Parameter>
		<Parameter Name="SendPort">1234</Parameter>
	</Parameter>
   </Module>

</ModuleConfigurations>
```

**NOTE:** In this example, we are using `14558` as the vehicle's UDP port to listen on. This will be matched with the SITL configuration in the next section. If this port is changed, make sure to reflect that change in the following JSON configuration file.

### <a name="sitl-config"></a> SITL Configuration
In the `ardupilot/Tools/autotest/MACEConfigs` directory on the Host machine, open the `sample_1Vehicle.xml` file. Any modifications to this file will be reflected to the container in the directory specified in the run command above. Modify the `vehicles` section to match the following, making sure to substitute your network settings where necessary:

```json
"vehicles": {
    "1": {
        "type": "ArduCopter",
        "frame": "quad",
        "sensor": "hokuyo_utm30lx",
        "enableSensor": true,
        "position": {
            "lat": 37.890425,
            "lng": -76.81191,
            "alt_msl": 0,
            "heading_deg": 240
        },
        "console": false,
        "simulated": true,
        "outputIP": "172.17.0.3",
        "outputPort": 14558,
        "commPort": ""
    }
}
```
**NOTE:** Again, in this example, we are using `14558` as the vehicle's UDP port to listen on. This should be matched with the MACE UDP configuration in the above section. If this port is changed, make sure to reflect that change in the MACE XML configuration file.

## <a name="sim-start"></a> Start the Simulation
Now that we have Docker containers built and configuration files set up properly, we can run the full simulation. First, start the GUI with two terminals:
```
**Terminal 1:**
$ yarn run watch

*** Terminal 2:**
$ yarn run start
```
(In Windows, you can run `yarn run launch` instead of using two terminals)

Now, start the MACE container and SITL containers in separate terminals:
```
**Terminal 1:**
$ docker run --name mace -v MACE/MaceSetup_Configs:MACE/MaceSetup_Configs -p 5678:5678 -it ubuntu1604:mace

*** Terminal 2:**
$ docker run --name sitl -v ardupilot/Tools/autotest/MACEConfigs/:ardupilot/Tools/autotest/MACEConfigs –it ubuntu1604:sitl
```

In the MACE container, run:
```
$ ./bin/MACE
```
Note that if you didn't modify the `Default.xml` file with your Network Settings, point MACE to the correct configuration file using `./bin/MACE <path>/<to>/<configuration_file>`, where `<path>/<to>/<configuration_file>` is replaced with the path to your correct config file.

After MACE has started, start the SITL instance in the SITL container:
```
$ python sim_multi_vehicle.py MACEConfigs/sample_1Vehicle.json
```

If your configurations are set up properly, you should see a vehicle start up in the SITL container, MACE will respond to the new vehicle, and your GUI will display a vehicle. You should be able to command the vehicle via the GUI.


# <a name="todo"></a> TODO
- Docker tips
  - Detach vs. Kill
  - Docker ps
  - Start/attach to detached container


