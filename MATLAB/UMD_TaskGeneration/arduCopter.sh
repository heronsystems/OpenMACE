#!/bin/bash
gnome-terminal -e "bash -c \"sim_vehicle.py -I 1 --out=udp:127.0.0.1:14551 --custom-location=38.97369900,-76.92189697,0,240\""
gnome-terminal -e "bash -c \"sim_vehicle.py -I 2 --out=udp:127.0.0.1:14552 --custom-location=38.97370015,-76.92182775,0,240\""
