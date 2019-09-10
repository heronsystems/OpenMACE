# UPDATE_BATTERY
# Each drone will issue this update periodically (e.g., 1Hz) to indicate current battery status

time    timestamp
uint8   vehicleID
uint16  voltage
uint16  current
uint16  percentRemaining