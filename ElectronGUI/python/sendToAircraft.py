from AircraftGC import AircraftHandler
import numpy as np
import sys
import json

# Read in arguments:
argWaypoints = np.asarray(json.loads(sys.argv[1]))

print argWaypoints

# newObject = AircraftHandler(argWaypoints, 20)
# newObject.runCommand()
