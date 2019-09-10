# UPDATE_VEHICLE_TARGET
# Each drone will issue this update periodically as it is progressing towards a target location.

time    timestamp
uint8   vehicleID
float64 northing
float64 easting
float64 altitude
float64 distance
uint8 state
