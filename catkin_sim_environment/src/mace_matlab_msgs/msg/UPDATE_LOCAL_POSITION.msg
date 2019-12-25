# UPDATE_LOCAL_POSITION
# Each drone will issue this update periodically (e.g., 1 Hz) to relay position

time    timestamp
uint8   vehicleID
uint8   frame
float64 northing
float64 easting
float64 altitude
float64 northSpeed
float64 eastSpeed
float64 altSpeed
