#### <a name="building-on-odroid"></a> Building On ODROID

For ODROID, QT will need to be compiled from source due to archetecture.

Throughout these instructions %VERSION% will refer to the version of Qt installing, for me it was 5.7.0, but it may be different depending on your preference.

Install Qt onto the host machine by downloading Qt source from  
`https://www.qt.io/download-open-source/`

The following instructions are largely based on Qt's own instructions: [http://doc.qt.io/qt-5/linux-building.html](http://doc.qt.io/qt-5/linux-building.html).

Prior to building ensure libxcb is installed by running:
```
apt-get install libxcb1-dev
apt-get install libx11-dev
```

To build Qt Tool chain navigate a shell to the unziped folder and run
```
./configure -opensource -confirm-license -qt-xcb
make
make install
```
Next add 
```
PATH=/usr/local/Qt-%VERSION%/bin:$PATH
export PATH
```
to `.profile`, after this change log-off/log-on would be required. After this is finished, you can follow the steps above to build with the Qt Creator IDE.



#### <a name="running-on-odroid"></a> Running on ODROID:
_NOTE_: If you cannot run via command line on an ODROID, try running the following command:

`export LD_LIBRARY_PATH=<path_to_MACE>/lib`

where `<path_to_MACE>` is replaced with the local path to your MACE root directory. This tells the console when running an application to also look in that path for any applicable libraries. 