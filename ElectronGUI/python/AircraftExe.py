#!/usr/bin/env python
import sys, struct, time, os, threading, math
import dronekit_sitl
import logging
from pymavlink.dialects.v20 import common as mavlink
from dronekit import connect, VehicleMode, Command, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from argparse import ArgumentParser
from threading import Thread

LOG_FILENAME = 'AircraftTesting.log'
logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)

logging.debug("This is the begginning")

missionLength = 0
currentMissionIndex = 0

latArray = [35.738618,35.73825,35.738251,35.738618,35.738618,35.738253,35.738254,35.738618,35.738618,35.738255,35.738256,35.738618]
lngArray = [-78.847804,-78.847814,-78.847739,-78.847729,-78.847654,-78.847664,-78.847589,-78.847579,-78.847504,-78.847514,-78.847439,-78.847429]
altArray = [5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0]

stopThread = True
last_mode_cache = "STABILIZE"

def performMissionFunciton():
    cmds = vehicle.commands
    cmds.clear()
    for i in range(0,len(latArray),1):
        cmd1 = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, latArray[i], lngArray[i], altArray[i])
        cmds.add(cmd1)
    cmds.upload()                 
    vehicle.airspeed = 3

    for i in range(0, len(latArray), 1):
        if stopThread == True:
            break
        else:
            goto(latArray[i], lngArray[i], altArray[i], 3)

missionThread = Thread(target=performMissionFunciton)

def vehicleModeCallback(self, attr_name, value):
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.
    global last_mode_cache
    global stopThread
    global missionThread
    # Only publish when value changes
    if value!=last_mode_cache:
        print " CALLBACK: Mode changed to", value
        last_mode_cache=value
        if(value == "GUIDED"):
            print "The mode was now in guided"
            stopThread = False
            missionThread = Thread(target=performMissionFunciton)
            missionThread.start()
        else:
            if missionThread.isAlive():
                stopThread = True
                missionThread.join()
                print "The mission thread was stopped"
            print "We have changed into something other than guided"

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def goto(latitude, longitude, altitude, radius):
    global stopThread
    print "I am in the go to function"
    targetLocation = LocationGlobal(latitude, longitude, altitude)
    goto_position_target_global_int(targetLocation)

    while stopThread == False:  # Stop action if we are no longer in guided mode.
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        #goto_position_target_global_int(targetLocation)
        if remainingDistance <= radius:  # Just below target, in case of undershoot.
            print "Reached target"
            break;
        #time.sleep(1)

def goto_position_target_global_int(targetLocation):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        targetLocation.lat * 1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        targetLocation.lon * 1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        targetLocation.alt,
        # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0,  # X velocity in NED frame in m/s
        0,  # Y velocity in NED frame in m/s
        0,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def arm_and_takeoff(targetAltitude):
    while not vehicle.is_armable:
        print("The vehicle cannot arm in this state")
        return
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for the vehicle to arm")
        time.sleep(0.5)

    print "I have told the vehicle to takeoff to an altitude of %s" %targetAltitude
    vehicle.simple_takeoff(targetAltitude)

parser = ArgumentParser(description=__doc__)
parser.add_argument("--baudrate", type=int,
                  help="master port baud rate", default=57600)
parser.add_argument("--device", help="serial device", default="/dev/ttyUSB0")
parser.add_argument("--rate", default=4, type=int, help="requested stream rate")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                  default=255, help='MAVLink source system for this GCS')
parser.add_argument("--showmessages", action='store_true',
                  help="show incoming messages", default=True)
args = parser.parse_args()

def heartbeatTimer(m, interval):
    mavlinkMSG = mavlink.MAVLink(m)
    while True:
        time.sleep(interval)
        mavlinkMSG.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, mavlink.MAV_STATE_ACTIVE)

        home = LocationGlobal(vehicle.home_location.lat, vehicle.home_location.lon, vehicle.home_location.alt)
        mavlinkMSG.mission_item_send(0, 0, 0, 0, 0, 0, 0, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt, vehicle.heading, vehicle.home_location.lat, vehicle.home_location.lon, vehicle.home_location.alt)

def handleMissionMessage(m):
    mavlinkMSG = mavlink.MAVLink(m)
    global vehicle
    global currentMissionIndex
    global latArray
    global lngArray
    global altArray
    global missionThread
    global stopThread

    '''show incoming mavlink messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            continue

        msgID = msg.get_msgId()
        if msgID == mavlink.MAVLINK_MSG_ID_MISSION_ITEM:
            if(msg.command == mavlink.MAV_CMD_NAV_WAYPOINT):
                index = msg.seq + 1
                logging.debug("I saw a new mission item at index %s",index)
                latArray.insert(index, msg.x)
                lngArray.insert(index, msg.y)
                altArray.insert(index, msg.z)
                currentMissionIndex = currentMissionIndex + 1
                if currentMissionIndex < missionLength:
                    logging.debug("Making a request for the item at %s",currentMissionIndex)
                    mavlinkMSG.mission_request_send(m.target_system, m.target_component, currentMissionIndex)
                else:
                    logging.debug("I am done requesting mission items")
                    # This will clear the current commands inside the vehicle for auto mode
                    cmds = vehicle.commands
                    cmds.clear()
                    for i in range(0,len(latArray),1):
                        mavlink.MAVLink_command_long_message
                        cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, latArray[i], lngArray[i], altArray[i])
                        cmds.add(cmd1)
                    cmds.upload()
            elif(msg.command == mavlink.MAV_CMD_NAV_TAKEOFF):
                arm_and_takeoff(20)
            elif(msg.command == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH):
                vehicle.mode = VehicleMode("RTL")
            elif(msg.command == mavlink.MAV_CMD_NAV_LAND):
                vehicle.mode = VehicleMode("LAND")
            elif(msg.command == mavlink.MAV_CMD_DO_GUIDED_MASTER):
                print "I have been allowed to perform my guided mode"
                if missionThread.isAlive():
                    print "I am already in the mission mode"
                else:
                    vehicle.mode = VehicleMode("GUIDED")
                    stopThread = False
                    missionThread = Thread(target=performMissionFunciton)
                    missionThread.start()
        elif msgID == mavlink.MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            logging.debug("I saw a clear all message")
            vehicle.mode = VehicleMode("LOITER")
            if missionThread.isAlive():
                stopThread = True
                missionThread.join()
            latArray = []
            lngArray = []
            altArray = []
            #probably want to halt motion and wait for further commands as well
        elif msgID == mavlink.MAVLINK_MSG_ID_MISSION_COUNT:
            logging.debug("I saw a mission count message")
            missionLength = msg.count
            currentMissionIndex = 0
            mavlinkMSG.mission_request_send(m.target_system, m.target_component, currentMissionIndex)
        elif  msgID == mavlink.MAVLINK_MSG_ID_BAD_DATA:
            if(mavutil.all_printable(msg.data)):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            print(msg)

time.sleep(10)
if __name__ == '__main__':
    #vehicle = connect("udp:0.0.0.0:14550", wait_ready=False)
    vehicle = connect("/dev/ttySAC0", baud=57600, wait_ready=False)
    vehicle.add_attribute_listener('mode', vehicleModeCallback)

    cmds = vehicle.commands
    cmds.clear()
    # Get Vehicle Home location - will be `None` until first set by autopilot
    while not vehicle.home_location:
        if not vehicle.home_location:
            print " Waiting for home location ..."
            time.sleep(1)
    # We have a home location, so print it!
    print "\n Home location: %s" % vehicle.home_location

    master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

    mavlinkMSG = mavlink.MAVLink(master)
    heartbeatThread = Thread(target = heartbeatTimer, args = (master,1))
    heartbeatThread.start()

    handleMissionMessage(master)
