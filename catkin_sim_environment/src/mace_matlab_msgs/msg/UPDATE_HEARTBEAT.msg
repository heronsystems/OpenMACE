# UPDATE_HEARTBEAT
# Each drone will issue this update periodically (e.g., 1Hz) to indicate connectivity

time    timestamp
uint8   vehicleID
string  autopilot
string  aircraftType
bool    isCompanion
string   missionState 

## Mission States:
# MESTATE_UNEXECUTED = 0, 
# MESTATE_EXECUTING = 1,
# MESTATE_PAUSED = 2, 
# MESTATE_COMPLETED = 3, 
# MESTATE_ABORTED = 4

## TODO:
# uint8   currentCmdID
# uint8   wifiStatus
# uint8   radioStatus
# uint8   rtkStatus