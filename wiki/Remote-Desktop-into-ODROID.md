## Table of Contents
- [ODROID Steps](#odroid)
- [Windows Steps](#windows)
- [Restart over SSH](#restart-ssh)

## <a name="odroid"></a> _On ODROID (remote machine)_
On the ODROID, we need to install a VNC server manager and setup the permissions. For this, we will use `vino` and `dconf-tools`. To install them, connect the ODROID to a monitor and boot it up. Once booted, log in, open a terminal and run:
```
$ sudo apt-get install vino dconf-tools
```
Once installed, launch the vino preferences editor by running `vino-preferences`. This will launch the preferences editor window. In the **Sharing** section, make sure both _Allow other users to view your desktop_ and _Allow other users to control your desktop_ are checked. In the **Security** section, un-check every option. In the **Show Notification Area Icon** section, make sure _Only when someone is connected_ is selected. Close the window

Next, run `dconf-editor` in the terminal to launch the configuration editor. Navigate to `org`-->`gnome`-->`desktop`-->`remote-access`. Un-check _require-encryption_ and delete the default _vnc-password_ field. Close the window.

To start the VNC server, simply run `/usr/lib/vino/vino-server` in a terminal window. At this point, the server should be running and you can disconnect your ODROID from the monitor. If at any point the VNC server goes down, you can restart it from the remote Windows machine using PuTTY or other SSH manager (see the 
[Restart over SSH](#restart-ssh) section).

## <a name="windows"></a> _On Windows (viewing machine)_
To view the remote desktop running a VNC Server, download 
[VNC Viewer](https://www.realvnc.com/download/viewer/). Once downloaded, start the VNC Viewer application and simply search for the IP Address of the remote machine. Hit `Enter` and a separate window should pop up trying to connect to the remote desktop. Assuming it is successful, a window will appear giving you access to the ODROID's desktop.

## <a name="restart-ssh"></a> _Restart VNC server over SSH_
If the VNC server ever stops on the ODROID, you can restart the vino server over SSH. Using any SSH client ([PuTTY](http://www.putty.org/) is common), start an SSH session to the IP address of the ODROID. The default settings should be fine. When the SSH session opens, you will be prompted for the user name you wish to log in as and then the user's password. After entering those, you will have access to a terminal. Here, we will restart the vino VNC server by running the following commands:
```
$ export DISPLAY=:0.0
$ /usr/lib/vino/vino-server
```
You should see the server start and should be able to connect as normal using the VNC Viewer from your Windows machine. Note that if you close the SSH session or stop its vino server, your connection to the remote desktop will be lost and you will need to restart the vino server again.  
