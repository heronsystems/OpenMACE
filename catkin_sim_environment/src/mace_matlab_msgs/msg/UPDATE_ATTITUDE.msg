# UPDATE_ATTITUDE
# Each drone will issue this update periodically (e.g., 1 Hz) to relay orientation

time    timestamp
uint8   vehicleID
float64 roll
float64 pitch
float64 yaw
float64 yawRate
float64 pitchRate
float64 rollRate