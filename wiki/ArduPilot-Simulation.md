# Table of Contents
- [Installation](#installation)
  - [Windows](#windows)
  - [Linux](#linux)
- [Usage](#usage)
- [Setting up for MACE communications](#mace-comms)

This page details the installation and usage of the ArduPilot Software-in-the-Loop (SITL) simulation software. This software is open source and is located at: http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html. This simulation uses MAVProxy, and useful tips for usage can be found  here: [Copter SITL/MAVProxy Tutorial](http://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html) and here: [Using SITL for ArduPilot Testing](http://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)

# <a name="installation"></a> Installation
- [Windows](#windows)
- [Linux](#linux)

## <a name="windows"></a> Windows installation
The instructions found on the [Windows installation](http://ardupilot.org/dev/docs/sitl-native-on-windows.html) page on the ArduPilot SITL website are duplicated here with modifications for the Heron Systems fork.

### <a name="windows-prereqs"></a> Prerequisites for Windows installation
The ArduPilot SITL simulation requires `MAVProxy`, `Cygwin`, and `JSBSim` to be installed to function properly. There are two ways to install them. Instructions for both are found below, choose which methods suits you best. Note that the manual installation method is more robust.

- [Automated install](#prereqs-automated)
- [Manual install](#prereqs-manual)

#### <a name="prereqs-automated"></a> Automated install
Copy the following script into a PowerShell script (save as a `install.ps1`). Execute the script in a PowerShell terminal and once finished, skip to the [Building ArduPilot SITL](#build-sitl) section. If PowerShell is not installed, follow the instructions [HERE](https://docs.microsoft.com/en-us/powershell/scripting/setup/installing-windows-powershell?view=powershell-6) to install. 

<details><summary>install.ps1</summary>

```powershell
#Powershell script to download and configure the APM SITL environment

Import-Module BitsTransfer

Write-Output "Starting Downloads"

Write-Output "Downloading MAVProxy (1/7)"
Start-BitsTransfer -Source "http://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe" -Destination "$PSScriptRoot\MAVProxySetup-latest.exe"

Write-Output "Downloading Cygwin x64 (2/7)"
Start-BitsTransfer -Source "https://cygwin.com/setup-x86_64.exe" -Destination "$PSScriptRoot\setup-x86_64.exe"

Write-Output "Installing Cygwin x64 (3/7)"
Start-Process -wait -FilePath $PSScriptRoot\setup-x86_64.exe -ArgumentList "--root=C:\cygwin --no-startmenu --local-package-dir=$env:USERPROFILE\Downloads --site=http://cygwin.mirror.constant.com --packages autoconf,automake,ccache,gcc-g++,git,libtool,make,gawk,libexpat-devel,libxml2-devel,python2,python2-future,python2-libxml2,python2-pip,libxslt-devel,python2-devel,procps-ng,zip,gdb,ddd --quiet-mode"

Write-Output "Copying JSBSim and APM install script to Cygwin (4/7)"
Start-BitsTransfer -Source "https://github.com/ArduPilot/ardupilot/raw/master/Tools/autotest/win_sitl/jsbsimAPM_install.sh" -Destination "C:\cygwin\home\jsbsimAPM_install.sh"

Write-Output "Downloading extra Python packages (5/7)"
Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c 'pip2 install empy'"

Write-Output "Downloading and installing JSBSim, then downloading APM source (6/7)"
Start-Process -wait -FilePath "C:\cygwin\bin\bash" -ArgumentList "--login -i -c ../jsbsimAPM_install.sh"

Write-Output "Installing MAVProxy (7/7)"
& $PSScriptRoot\MAVProxySetup-latest.exe /SILENT | Out-Null

Write-Host "Finished. Press any key to continue ..."
$x = $host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
```

</details>

***

#### <a name="prereqs-manual"></a> Manual install
##### <a name="install-mavproxy"></a> MAVProxy
MAVProxy is a fully-functioning but minimalist console-based GCS that is commonly used for testing and developing ArduPilot:

- [Download MAVProxy for Windows](http://firmware.ardupilot.org/Tools/MAVProxy/MAVProxySetup-latest.exe) (latest build)
- Install the executable, accepting the license and all other default settings.

Other builds can be obtained from [http://firmware.ardupilot.org/Tools/MAVProxy/](http://firmware.ardupilot.org/Tools/MAVProxy/).

##### <a name="install-cygwin"></a> Cygwin
[Cygwin](http://www.cygwin.com/) provides the tools and libraries that allow us to rebuild ArduPilot on Windows

1. Download and run the [Cygwin 32-bit installer](https://cygwin.com/setup-x86.exe) or the [Cygwin 64-bit installer](https://cygwin.com/setup-x86_64.exe). 
2. Accept all the prompts (including default file locations) until you reach the _Select Packages_ dialog. There are thousands of packages. The easiest way to find the packages is to search on the name. When you've found a needed package, click on the **Skip** button to select it for download:

![cygwin_skip](https://github.com/heronsystems/MACE/blob/master/docs/images/cygwin_skip.png)

3. Select the packages listed below (search using the text in the "Name" field):

<table border="1" class="docutils">
<colgroup>
<col width="16%" />
<col width="84%" />
</colgroup>
<thead valign="bottom">
<tr class="row-odd"><th class="head">Name</th>
<th class="head">Category / Name / Description</th>
</tr>
</thead>
<tbody valign="top">
<tr class="row-even"><td>autoconf</td>
<td>Devel | autoconf: Wrapper scripts for autoconf commands</td>
</tr>
<tr class="row-odd"><td>automake</td>
<td>Devel | automake: Wrapper scripts for automake and aclocal</td>
</tr>
<tr class="row-even"><td>ccache</td>
<td>Devel | ccache: A C compiler cache for improving recompilation</td>
</tr>
<tr class="row-odd"><td>g++</td>
<td>Devel | gcc-g++ GNU Compiler Collection (C++)</td>
</tr>
<tr class="row-even"><td>git</td>
<td>Devel | git: Distributed version control system</td>
</tr>
<tr class="row-odd"><td>libtool</td>
<td>Devel | libtool: Generic library support script</td>
</tr>
<tr class="row-even"><td>make</td>
<td>Devel | make: The GNU version of the &#8216;make&#8217; utility</td>
</tr>
<tr class="row-odd"><td>gawk</td>
<td>Interpreters | gawk: GNU awk, a pattern scanning and processing language</td>
</tr>
<tr class="row-even"><td>libexpat</td>
<td>Libs | libexpat-devel: Expat XML parser library (development files)</td>
</tr>
<tr class="row-odd"><td>libxml2-devel</td>
<td>Libs | libxml2-devel: Gnome XML library (development)</td>
</tr>
<tr class="row-even"><td>libxslt-devel</td>
<td>Libs | libxslt-devel: XML template library (development files)</td>
</tr>
<tr class="row-odd"><td>python2-devel</td>
<td>Python | python2-devel: Python2 language interpreter (python3 does not work yet)</td>
</tr>
<tr class="row-even"><td>procps</td>
<td>System | procps-ng: System and process monitoring utilities (required for pkill)</td>
</tr>
<tr class="row-even"><td>patch</td>
<td>Devel | patch: Applies diff files</td>
</tr>
<tr class="row-odd"><td>cmake</td>
<td>Devel | cmake: Cross-platform makefile generation system</td>
</tr>
<tr class="row-even"><td>flex</td>
<td>Devel | flex: A fast lexical analizer generator</td>
</tr>
<tr class="row-odd"><td>bison</td>
<td>Devel | bison: GNU yacc-compatible parser generator</td>
</tr>
<tr class="row-even"><td>zip</td>
<td>Devel | zip: Info-ZIP compression utility</td>
</tr>
<tr class="row-odd"><td>unzip</td>
<td>Devel | unzip: Info-ZIP decompression utility</td>
</tr>
<tr class="row-even"><td>python2-pip</td>
<td>Python | python2-pip: Python package instalation tool</td>
</tr>
</tbody>
</table>

4. When all the packages listed above are selected, click through the rest of the prompts and accept all other default options (including the additional dependencies).
5. Select **Finish** to start downloading the files.
6. If you want to compile the firmware as well, then you need to open a cygwin terminal and run the following commands:
```bash
pip2 install argparse
pip install empy
```

With everything installed, now we want to set up directories and paths in Cygwin. To do so:

1. Open and then close Cygwin Terminal from the desktop or start menu icon. 
      - This will create initialization files for the user in the Cygwin home directory (and display their locations). For example, a user's home directory might be located at `C:\cygwin\home\user_name\`. 
2. Navigate the file system to the home directory and open the `.bashrc` file (e.g. `C:\cygwin\home\user_name\.bashrc`).
3. Add the path to your ArduPilot source to cygwin by adding the following line to the end of `.bashrc`
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
```
Note that your source may not be in **$HOME** but in some other fixed path that starts with `/cygdrive/c/Users`. 

Close and open the Cygwin terminal for the changes to take effect. The **.bashrc** file will be loaded next time you open the Cygwin terminal.

##### <a name="install-python"></a> Python Packages
Open a Cygwin Terminal and run the following commands to install required python packages:
```
python -m ensurepip --user
python -m pip install --user future
python -m pip install --user lxml
python -m pip install --user uavcan
```

### <a name="build-sitl"></a> Building ArduPilot SITL
Open (or reopen) a Cygwin Terminal and clone the Heron Systems Github fork of the ArduPilot repository:
```
git clone git://github.com/heronsystems/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

In the terminal, navigate to the `ArduCopter` directory and run **make** as shown:
```
cd ~/ardupilot/ArduCopter
make sitl -j4
```
Note that the platform that is built depends on the directory where you run make (so this this will build _Copter_).


***

## <a name="linux"></a> Linux installation
The instructions found on the [Linux installation](http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html) page on the ArduPilot SITL website are duplicated here with modifications for the Heron Systems fork.

### <a name="linux-clone"></a> Clone the Heron Systems fork
Open a terminal and navigate to the directory where you would like to clone the ArduPilot repository and run:
```
git clone https://github.com/heronsystems/ardupilot
cd ardupilot
git submodule update --init --recursive
```

### <a name="linux-packages"></a> Install some required packages
If you are on a debian based system (such as Ubuntu or Mint, run the following script to install packages automatically:
```
Tools/scripts/install-prereqs-ubuntu.sh -y
```

Make sure that the following two lines are included in the bottom of your `~/.bashrc` file:

```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Reload the path (log out and log back in to make permanent):
```
. ~/.profile
```


### <a name="linux-build"></a> Building ArduPilot SITL
In a terminal window, navigate to the root `ardupilot` directory. Run the following to build SITL:
```
./waf configure
./waf copter
```
Note for full build options and settings, see the [ArduPilot BUILD.md](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md) file.


***

# <a name="usage"></a> Usage
To launch the simulation, open a terminal (note: for Windows, open a Cygwin terminal) and change directories into the vehicle type that you wish to launch. To date, this simulation has only been tested with the ArduCopter platform, but theoretically it will work with ArduPlane as well. 

To launch the simulation, we can simply change directory into your ArduCopter directory and launch the sim_vehicle.py script:

```
cd ardupilot/ArduCopter
sim_vehicle.py -I 2
```

* In the above code, `-I 2` tells the simulation to load the `copter_2.parm` file and set the ID to 2. Change this number to change the vehicle ID

Note that to allow for setting the vehicle ID via command line, we had to create new parameters files that explicitly set the ID. These are located in the `ardupilot/Tools/autotest/default_params/MACE_params/` directory. The `MACE_params` directory contains several files named `copter_N.parm`, where N is the vehicle ID number. If you open up one of those files, you should see at the bottom a line that sets the `SYSID_THISMAV` parameter. For example, for sysID 3, you would open `copter_3.parm` and see at the bottom: 

`SYSID_THISMAV    3`

If you want to add more vehicles, simply copy one of these files and rename it to reflect the vehicle ID you will be setting. Also, change the `SYSID_THISMAV` parameter in the file to reflect the new vehicle ID.


# <a name="mace-comms"></a> Setting up for MACE communications
By default, the simulation outputs data over a machine's loopback address (i.e. 127.0.0.1) on ports 14550 and 14551. Regardless of whether or not your simulation is running on the same machine as your MACE instance, you will need to add an output destination and port number to the simulation. Find your machine's IP address and start the simulation with the `-m --out` flag:

`sim_vehicle.py -m --out=udp:<your ip address here>:<port number here> -I 2`

Replace `<your ip address here>` with your IP address and `<port number here>` with a port that you wish to receive data on. Typically, this port number is set to 14550 + N, where N is your vehicle ID. 

Finally, it is sometimes useful to use the MAVProxy console to visualize what is happening with the simulation. To do so, use the `--console` flag with launching the simulation. Note that this will launch a separate window from your terminal. While helpful, as you launch more simulations, the extra windows become cumbersome. Work is being done to consolidate these consoles.  

An example command to launch the simulation with the console, output to a machine via UDP, and set the vehicle ID is as follows:

`sim_vehicle.py --console -m --out=udp:192.168.1.33:14552 -I 2`


_One final note:_ you cannot launch two simulation instances with the same ID. The simulation will not crash, however, the last simulation that is launched with a duplicate ID will supersede all previous simulations. In other words, the other simulations with that ID will enter a "Link Down" state and will not be controllable. You will, however, still be able to control the last simulation that was launched.  
