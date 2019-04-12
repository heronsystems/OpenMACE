from AircraftGC import AircraftHandler
from threading import Thread
import numpy as np
import sys, json


# argboundaryVertices = np.asarray(json.loads(sys.argv[1]))
# arghotSpotsIn = np.asarray(json.loads(sys.argv[2]))
# argPVal = json.loads(sys.argv[3])
# argGridVal = json.loads(sys.argv[4])
# argPathDirection = json.loads(sys.argv[5])


positionArray = [[[37.890688, -76.815885], [37.890955, -76.815424], [37.891006, -76.814659]], [[37.90, -76.815885], [37.90, -76.815424], [37.90, -76.814659]]]

newObject = AircraftHandler(positionArray, 20)
aircraftThread = Thread(target=newObject.runCommand)
aircraftThread.start()
# newObject.runCommand()

# print positionArray
