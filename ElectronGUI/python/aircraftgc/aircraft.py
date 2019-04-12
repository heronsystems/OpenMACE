from __future__ import print_function
from dronekit import LocationGlobal
from pymavlink import mavutil, mavwp
from pymavlink.dialects.v20 import common as mavlink


class Aircraft:
    def __init__(self, comm_port):
        # create a mavlink serial instance
        self.mav_connection = mavutil.mavlink_connection(comm_port, baud=57600)
        self.mav_link = mavlink.MAVLink(self.mav_connection)
        self.home_location = None
        self.location = None
        self.in_mission = False
        self.heading = 0

    def set_mission_from_positions(self, position_targets, altitude=20, radius=5, frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT):
        waypoint_loader = mavwp.MAVWPLoader()

        for position_ind, position in enumerate(position_targets):
            print("The current target position is %s" % position_ind)
            waypoint = mavlink.MAVLink_mission_item_message(self.mav_connection.target_system,
                                                            self.mav_connection.target_component,
                                                            position_ind, frame, mavlink.MAV_CMD_NAV_WAYPOINT,
                                                            0, 0, 0, radius, 0, 0,
                                                            position[0], position[1],
                                                            altitude)
            waypoint_loader.add(waypoint)

        self.clear_mission()
        self.mav_link.mission_count_send(self.mav_connection.target_system,
                                         self.mav_connection.target_component,
                                         len(position_targets))

        print("I am done telling how many targets there are {0}".format(len(position_targets)))
        self.handle_mission_transmission(waypoint_loader)

    def handle_mission_transmission(self, waypoint_loader):
        self.in_mission = True
        while True:
            msg = self.mav_connection.recv_match(blocking=True)
            if not msg:
                return

            msg_id = msg.get_msgId()
            if msg_id == mavlink.MAVLINK_MSG_ID_MISSION_REQUEST:
                print("New mission request object for: %u" % msg.seq)
                mission_index = msg.seq
                self.mav_link.send(waypoint_loader.wp(mission_index))
                if mission_index == waypoint_loader.count() - 1:
                    print("I am done transmitting for this aircraft")
                    break
            elif msg_id == mavlink.MAVLINK_MSG_ID_BAD_DATA:
                if mavutil.all_printable(msg.data):
                    print('Bad data for mission', msg.data)
            else:
                print(msg)
        self.in_mission = False

    def clear_mission(self):
        self.mav_link.mission_clear_all_send(self.mav_connection.target_system,
                                             self.mav_connection.target_component)
        print(self, "I am done sending the clear all")

    def takeoff(self):
        self.transmitCommand(mavlink.MAV_CMD_NAV_TAKEOFF)

    def land(self):
        self.transmitCommand(mavlink.MAV_CMD_NAV_LAND)

    def guided(self):
        self.transmitCommand(mavlink.MAV_CMD_DO_GUIDED_MASTER)

    def RTL(self):
        self.transmitCommand(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def transmitCommand(self, mav_command):
        self.mav_link.mission_item_send(0, 0, 0, 0, mav_command, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def send_ack(self):
        self.mav_link.command_ack_send(0, 0)

    def wait_heartbeat(self):
        self.mav_connection.wait_heartbeat()
        print("Heartbeat from APM (system %u component %u)" % (self.mav_connection.target_system, self.mav_connection.target_system))

    def wait_set_home_location(self):
        msg = self.mav_connection.recv_match(type="MISSION_ITEM", blocking=True)
        # 2, 3, 4 aircraft lat lon alt
        self.location = LocationGlobal(msg.param1, msg.param2, msg.param3)
        self.heading = msg.param4
        self.home_location = LocationGlobal(msg.x, msg.y, msg.z)
        return self.home_location

    def get_location(self):
        if not self.in_mission:
            msg = self.mav_connection.recv_match(type="MISSION_ITEM", blocking=True)
            # 2, 3, 4 aircraft lat lon alt
            self.location = LocationGlobal(msg.param1, msg.param2, msg.param3)
            self.heading = msg.param4
            self.home_location = LocationGlobal(msg.x, msg.y, msg.z)
        return self.location, self.heading

    def disconnect(self):
        self.mav_connection.close()
        self.mav_connection = None
        self.mav_link = None
        self.home_location = None
