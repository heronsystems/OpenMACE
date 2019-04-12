import sys, struct, time, os, threading, math
from pymavlink.dialects.v20 import common as mavlink
from pymavlink.dialects.v10 import ardupilotmega as mavlinkArdupilot
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil, mavwp
from argparse import ArgumentParser
from pynput.keyboard import Key, KeyCode, Listener


class AircraftHandler(object):
    nAircraft = 1
    aircraftMsg = []
    aircraftHome = []
    aircraftComms = []
    aircraftID = []
    sortedAircraftOrder = []

    positionArray = []
    targetAltitude = 20
    latArray = []
    lngArray = []
    altArray = []

    def __init__(self, positionArray, targetAltitude):
        self.positionArray = positionArray
        self.targetAltitude = targetAltitude

    def sortWesternPoints(self, locationArray):
        newlonLocation = []
        lonArray = []
        for i in range(0, len(locationArray), 1):
            lonArray.append(locationArray[i].lon)
        newlonLocation = sorted(lonArray)

        arrayOrderSorted = []
        for i in range(0, len(newlonLocation), 1):
            for j in range(0, len(locationArray), 1):
                if newlonLocation[i] ==  locationArray[j].lon:
                    arrayOrderSorted.append(j)
                    break
        return arrayOrderSorted


    def wait_heartbeat(self, id, m):
        print("Waiting for APM heartbeat from aircraft %s" %id)
        m.wait_heartbeat()
        print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


    def wait_homeLocation(self, id, m):
        print("Waiting for APM home location from aircraft %s" % id)
        msg = m.recv_match(type="MISSION_ITEM", blocking=True)
        homeLocation = LocationGlobal(msg.x,msg.y,msg.z)
        print("Waiting for APM home location from aircraft %s" % id)
        self.aircraftHome.append(homeLocation)


    def handleMissionTransmission(self, m, wp):
        while True:
            mavlinkMSG = mavlink.MAVLink(m)
            msg = m.recv_match(blocking=True)
            if not msg:
                return

            msgID = msg.get_msgId()
            if msgID == mavlink.MAVLINK_MSG_ID_MISSION_REQUEST:
                print("New mission request object for: %u"%msg.seq)
                missionIndex = msg.seq
                mavlinkMSG.send(wp.wp(missionIndex))
                if missionIndex == wp.count() - 1:
                    print"I am done transmitting for this aircraft"
                    break

            elif  msgID == mavlink.MAVLINK_MSG_ID_BAD_DATA:
                if(mavutil.all_printable(msg.data)):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()

            elif msgID == mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                aircraftLatitude = msg.lat
                aircraftLongitude = msg.lon

            else:
                print(msg)


    def on_press(self, key):
        print 'on press'
        if key == Key.up:
            print "Perform a takeoff"
            self.transmitCommand(mavlink.MAV_CMD_NAV_TAKEOFF)
        elif key == Key.down:
            print "Perform a landing"
            self.transmitCommand(mavlink.MAV_CMD_NAV_LAND)
        elif key == Key.enter:
            print "Perform a guided mode"
            self.transmitCommand(mavlink.MAV_CMD_DO_GUIDED_MASTER)
        elif key == Key.backspace:
            print "Perform a RTL"
            self.transmitCommand(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def on_release(self, key):
        if key == Key.esc:
            return False

    def transmitCommand(self, command):
        for i in range(0,len(self.aircraftComms),1):
            transmitVehicle = self.sortedAircraftOrder[i]
            self.aircraftMsg[transmitVehicle].mission_item_send(0, 0, 0, 0, command, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def runCommand(self):

        # create a mavlink serial instance

        aircraftOne = mavutil.mavlink_connection('com4', baud=57600)
        self.aircraftComms.append(aircraftOne)
        self.aircraftMsg.append(mavlink.MAVLink(aircraftOne))

        # aircraftTwo = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600)
        # self.aircraftComms.append(aircraftTwo)
        # self.aircraftMsg.append(mavlink.MAVLink(aircraftTwo))

#        master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
#        mavlinkMSG = mavlink.MAVLink(master)
#       mavlinkMSG.command_long_send()

        # wait for the heartbeat msg to find the system ID
        for i in range(0, len(self.aircraftComms), 1):
            self.wait_heartbeat(i, self.aircraftComms[i])
            print "I saw a heartbeat, now I am going to acknowledge I have seen the aircraft"
            self.aircraftMsg[i].command_ack_send(0, 0)
            print "The value I am trying to put in is: %s" % i
            self.wait_homeLocation(i, self.aircraftComms[i])

        self.sortedAircraftOrder = self.sortWesternPoints(self.aircraftHome)

        for i in range(0, len(self.aircraftComms), 1):
            wp = mavwp.MAVWPLoader()
            seq = 1
            frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            radius = 5
            currentTargets = self.positionArray[i]

            for j in range(0, len(currentTargets), 1):
                currentPosition = currentTargets[j]
                print "The current position is %s" %currentPosition
                wp.add(
                    mavlink.MAVLink_mission_item_message(self.aircraftComms[i].target_system, self.aircraftComms[i].target_component,
                                                         i, frame, mavlink.MAV_CMD_NAV_WAYPOINT,
                                                         0, 0, 0, radius, 0, 0,
                                                         currentPosition[0], currentPosition[1], self.targetAltitude))
            transmitVehicle = self.sortedAircraftOrder[i]
            self.aircraftMsg[transmitVehicle].mission_clear_all_send(self.aircraftComms[transmitVehicle].target_system,
                                                                self.aircraftComms[transmitVehicle].target_component)
            print "I am done sending the clear all"
            self.aircraftMsg[transmitVehicle].mission_count_send(self.aircraftComms[transmitVehicle].target_system,
                                                            self.aircraftComms[transmitVehicle].target_component, len(currentTargets))
            print "I am done telling how many targets there are "
            self.handleMissionTransmission(self.aircraftComms[transmitVehicle], wp)
            # tell the vehicle to takeoff


        # Collect events until released
        keyboard_listener = Listener(on_press=self.on_press)

        # when done
        keyboard_listener.start()
        keyboard_listener.join()
