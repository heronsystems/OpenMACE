# UPDATE_CMD_STATUS
# Echo drone ID, command ID, and report status (=true if command was successfully issued).

time    timestamp
uint8   vehicleID
uint8   commandID
uint8   status # STATUS_EXECUTING = 0, STATUS_COMPLETE = 1, STATUS_ERROR = 2