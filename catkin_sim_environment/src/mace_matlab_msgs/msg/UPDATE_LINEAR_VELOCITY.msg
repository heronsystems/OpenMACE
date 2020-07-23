# UPDATE_LINEAR_VELOCITY
# Each drone will issue this update periodically (e.g., 1 Hz) to relay position

time    timestamp
uint8   vehicleID
uint8   frame
float64 northingSpeed
float64 eastingSpeed
float64 altSpeed
