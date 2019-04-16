The FCL library required by this project needs MinGW64 to compile.
There are likely a couple ways to get the correct toolchain, this document details how to download MinGW64 through MSYS2.

Download and install MSYS2 from `http://www.msys2.org/`.

In MSYS2 shell run the following commands to install proper tools:
```
pacman -S mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-gdb
pacman -S make
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw64/mingw-w64-x86_64-make
```

[Optional] Add MSYS tools to path:
```
C:\msys64\usr\bin
C:\msys64\mingw64\bin
```