#!/bin/bash

echo "Adding OpenMACE/lib library path..."
echo "${PWD}/lib" >> /etc/ld.so.conf.d/OpenMACE.conf

echo "Adding OpenMACE/tools/octomap/lib library path..."
echo "${PWD}/tools/octomap/lib" >> /etc/ld.so.conf.d/OpenMACE.conf

echo "Running ldconfig..."
ldconfig

echo "Setting OpenMACE environment variables..."
echo "export MACE_ROOT=${PWD}" >> ~/.bashrc
source ~/.bashrc
