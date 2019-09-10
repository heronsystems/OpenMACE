# UPDATE_HEARTBEAT
# Each drone will issue this update periodically (e.g., 1Hz) to indicate current GPS status

time    timestamp
uint8   vehicleID
string  gpsFix
uint16  numSats
uint16  hdop
uint16  vdop