# UPDATE_GEODETIC_POSITION
# Each drone will issue this update periodically (e.g., 1 Hz) to relay position

time    timestamp
uint8   vehicleID
uint8   frame
float64 latitude
float64 longitude
float64 altitude
float64 northSpeed
float64 eastSpeed
