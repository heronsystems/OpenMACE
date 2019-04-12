from __future__ import print_function
from dronekit import LocationGlobal
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
# from pymavlink.dialects.v10 import ardupilotmega as mavlinkArdupilot
from aircraft import Aircraft


class GroundControl:
    def __init__(self):
        self.aircrafts = {}

    def connect_to(self, comm_port):
        aircraft = Aircraft(comm_port)
        # wait for the heartbeat msg to find the system ID
        print("Waiting for APM heartbeat from aircraft %s" % comm_port)
        aircraft.wait_heartbeat()
        print("I saw a heartbeat, now I am going to acknowledge I have seen the aircraft")
        aircraft.send_ack()
        print("Waiting for APM home location from aircraft %s" % comm_port)
        aircraft.wait_set_home_location()
        self.aircrafts[comm_port] = aircraft

    def plan_mission(self, comm_port, mission_positions):
        # TODO: fancy planning stuff
        self.aircrafts[comm_port].set_mission_from_positions(mission_positions)

    def get_home_location(self, comm_port):
        location = self.aircrafts[comm_port].home_location
        return {"latitude": location.lat, "longitude": location.lon, "altitude": location.alt}

    def get_location(self, comm_port):
        location, heading = self.aircrafts[comm_port].get_location()
        return {"latitude": location.lat, "longitude": location.lon, "altitude": location.alt, "heading": heading}

    def takeoff(self, comm_port):
        self.aircrafts[comm_port].takeoff()

    def land(self, comm_port):
        self.aircrafts[comm_port].land()

    def guided(self, comm_port):
        self.aircrafts[comm_port].guided()

    def RTL(self, comm_port):
        self.aircrafts[comm_port].RTL()

    def disconnect(self, comm_port):
        self.aircrafts[comm_port].disconnect()
        del self.aircrafts[comm_port]

    #
    # def runCommand(self):
    #     # create a mavlink serial instance
    #     aircraftOne = mavutil.mavlink_connection('com4', baud=57600)
    #     self.aircraftComms.append(aircraftOne)
    #     self.aircraftMsg.append(mavlink.MAVLink(aircraftOne))
    #
    #     # wait for the heartbeat msg to find the system ID
    #     for i in range(0, len(self.aircraftComms), 1):
    #         self.wait_heartbeat(i, self.aircraftComms[i])
    #         print "I saw a heartbeat, now I am going to acknowledge I have seen the aircraft"
    #         self.aircraftMsg[i].command_ack_send(0, 0)
    #         print "The value I am trying to put in is: %s" % i
    #         self.wait_homeLocation(i, self.aircraftComms[i])
    #
    #     self.sortedAircraftOrder = self.sortWesternPoints(self.aircraftHome)
    #
    #     for i in range(0, len(self.aircraftComms), 1):
    #         wp = mavwp.MAVWPLoader()
    #         seq = 1
    #         frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    #         radius = 5
    #         currentTargets = self.positionArray[i]
    #
    #         for j in range(0, len(currentTargets), 1):
    #             currentPosition = currentTargets[j]
    #             print "The current position is %s" %currentPosition
    #             wp.add(
    #                 mavlink.MAVLink_mission_item_message(self.aircraftComms[i].target_system, self.aircraftComms[i].target_component,
    #                                                      i, frame, mavlink.MAV_CMD_NAV_WAYPOINT,
    #                                                      0, 0, 0, radius, 0, 0,
    #                                                      currentPosition[0], currentPosition[1], self.targetAltitude))
    #         transmitVehicle = self.sortedAircraftOrder[i]
    #         self.aircraftMsg[transmitVehicle].mission_clear_all_send(self.aircraftComms[transmitVehicle].target_system,
    #                                                             self.aircraftComms[transmitVehicle].target_component)
    #         print "I am done sending the clear all"
    #         self.aircraftMsg[transmitVehicle].mission_count_send(self.aircraftComms[transmitVehicle].target_system,
    #                                                         self.aircraftComms[transmitVehicle].target_component, len(currentTargets))
    #         print "I am done telling how many targets there are "
    #         self.handleMissionTransmission(self.aircraftComms[transmitVehicle], wp)
    #         # tell the vehicle to takeoff
    @staticmethod
    def wait_heartbeat(id, mav_connection):
        print("Waiting for APM heartbeat from aircraft %s" % id)
        mav_connection.wait_heartbeat()
        print("Heartbeat from APM (system %u component %u)" % (mav_connection.target_system, mav_connection.target_system))

    @staticmethod
    def wait_home_location(self, id, mav_connection):
        print("Waiting for APM home location from aircraft %s" % id)
        msg = mav_connection.recv_match(type="MISSION_ITEM", blocking=True)
        home_location = LocationGlobal(msg.x, msg.y, msg.z)
        print("Waiting for APM home location from aircraft %s" % id)
        return home_location
